#include "rrtstar.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "matplotlibcpp.h"
#include <map>
#include <stdexcept>
#include <Eigen/Dense> // Include Eigen library for matrix operations

// namespace plt = matplotlibcpp;


RRTStar::RRTStar(const std::array<double, 6>& start_q, const std::array<double, 6>& goal_q,
                 double map_width, double map_height, double map_depth,
                 double step_size, double neighbor_radius,
                 double safety_margin, int max_iter,
                 double min_x, double min_y, double min_z)
    : map_width(map_width), map_height(map_height), map_depth(map_depth),
      map_min_x(min_x), map_min_y(min_y), map_min_z(min_z),
      step_size(step_size), neighbor_radius(neighbor_radius),
      safety_margin(safety_margin), max_iter(max_iter),
      gen(std::random_device{}()),
      dis_x(min_x, min_x + map_width), 
      dis_y(min_y, min_y + map_height), 
      dis_z(min_z, min_z + map_depth),
      goal_config(goal_q),
      node_adapter(nodes) {
    
    if (!isStateValid(start_q) || !isStateValid(goal_q)) {
        throw std::invalid_argument("Start or goal configuration is invalid!");
    }

    start_node = std::make_unique<Node>(start_q);
    goal_node = std::make_unique<Node>(goal_q);

    nodes.push_back(start_node.get());
    kdtree = std::make_unique<nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, NodeAdapter>,
        NodeAdapter, 6>>(6, node_adapter, nanoflann::KDTreeSingleIndexAdaptorParams());
    kdtree->buildIndex();
}

RRTStar::~RRTStar() = default;

bool RRTStar::lineAABBIntersection(const std::array<double, 3>& start,
                                   const std::array<double, 3>& end,
                                   const std::array<double, 3>& box_min,
                                   const std::array<double, 3>& box_max) {
    double tmin = 0.0;
    double tmax = 1.0;
    std::array<double, 3> dir = {end[0] - start[0], end[1] - start[1], end[2] - start[2]};
    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < 1e-9) {
            if (start[i] < box_min[i] || start[i] > box_max[i]) {
                return false;
            }
        } else {
            double ood = 1.0 / dir[i];
            double t1 = (box_min[i] - start[i]) * ood;
            double t2 = (box_max[i] - start[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }
    return tmin <= 1.0 && tmax >= 0.0;
}

// Static obstacle initialization
// std::vector<Obstacle> RRTStar::obstacles = {
//     Obstacle({350, 350, 350}, {400, 400, 400})  // Example obstacle away from goal
//     // Obstacle({4.0, 4.0, 4.0}, {6.0, 6.0, 6.0})
// };

std::vector<Obstacle> RRTStar::obstacles = {
    Obstacle({2, 2, 2}, {4, 4, 4}),  // Near the start
    Obstacle({6, 6, 6}, {8, 8, 8})   // Near the goal
};

// std::vector<Obstacle> RRTStar::obstacles = {
//     // Add multiple obstacles in the workspace
//     Obstacle({1.0, 1.0, 0.0}, {1.5, 1.5, 0.5}),  // Small obstacle near start
//     Obstacle({3.0, 3.0, 0.0}, {4.0, 4.0, 1.0}),  // Medium obstacle in middle
//     Obstacle({7.0, 7.0, 0.0}, {8.0, 8.0, 1.5})   // Large obstacle near goal
// };

// std::vector<Node*> RRTStar::findPath() {
//     for (int i = 0; i < max_iter; ++i) {
//         auto random_node = getRandomNode(i);
//         Node* nearest_node = nearest(random_node.get());
//         auto new_node = steer(nearest_node, random_node.get());

//         // Verify step size before proceeding
//         double step_dist = distance(nearest_node, new_node.get());
//         if (step_dist >= step_size) {  // Use >= to strictly enforce the limit
//             continue;
//         }
//         auto [colision_free, steps] = isCollisionFree(nearest_node, new_node.get());
//         if (colision_free) {
//             std::vector<Node*> neighbors = radiusSearch(new_node.get(), neighbor_radius);
//             Node* min_cost_node = nearest_node;
//             double min_cost = nearest_node->cost + distance(nearest_node, new_node.get());

//             // Choose parent with minimum cost
//             for (auto& neighbor : neighbors) {
//                 auto [neighbor_collision_free, _] = isCollisionFree(neighbor, new_node.get());
//                 if (neighbor_collision_free) {
//                     double tentative_cost = neighbor->cost + distance(neighbor, new_node.get());
//                     if (tentative_cost < min_cost) {
//                         min_cost = tentative_cost;
//                         min_cost_node = neighbor;
//                     }
//                 }
//             }

//             new_node->parent = min_cost_node;
//             new_node->cost = min_cost;
//             nodes.push_back(new_node.get());
//             node_storage.push_back(std::move(new_node));

//             // Rebuild KD-tree
//             kdtree->buildIndex();

//             // Rewire neighbors
//             rewire(neighbors, nodes.back());

//             // Check if we can reach goal
//             double dist_to_goal = distance(nodes.back(), goal_node.get());
//             if (dist_to_goal <= step_size) {
//                 auto [goal_collision_free, __] = isCollisionFree(nodes.back(), goal_node.get());
//                 if (goal_collision_free) {
//                     auto final_node = steer(nodes.back(), goal_node.get());
//                     auto [final_collision_free, ___] = isCollisionFree(nodes.back(), final_node.get());
//                     if (final_collision_free) {
//                         final_node->parent = nodes.back();
//                         final_node->cost = nodes.back()->cost + distance(nodes.back(), final_node.get());
//                         nodes.push_back(final_node.get());
//                         node_storage.push_back(std::move(final_node));
//                         break;
//                     }
//                 }
//             }
//         }
//     }

//     std::vector<Node*> path;
//     if (!nodes.empty()) {
//         Node* final_node = nodes.back();
//         if (distance(final_node, goal_node.get()) <= step_size) {
//             getFinalPath(final_node, path);
//         }
//     }

//     // visualizePath(path);

//     return path;
// }

std::vector<Node*> RRTStar::findPath() {
    auto path = globalPlanner();
    if (path.empty()) {
        std::cout << "Initial planning failed, retrying..." << std::endl;
        path = globalPlanner();
    }
    
    if (!path.empty()) {
        std::cout << "Initial path size: " << path.size() << "\n";
        auto initial_metrics = evaluatePathQuality(path);
        std::cout << "Initial path metrics:" << std::endl;
        std::cout << "Total length: " << initial_metrics.total_length << std::endl;
        std::cout << "Max step: " << initial_metrics.max_step << std::endl;
        std::cout << "Average step: " << initial_metrics.avg_step << std::endl;
        std::cout << "Smoothness: " << initial_metrics.smoothness << std::endl;
        
        if (path.size() > 2) {
            optimizePath(path);
            auto final_metrics = evaluatePathQuality(path);
            std::cout << "\nFinal path metrics:" << std::endl;
            std::cout << "Total length: " << final_metrics.total_length << std::endl;
            std::cout << "Max step: " << final_metrics.max_step << std::endl;
            std::cout << "Average step: " << final_metrics.avg_step << std::endl;
            std::cout << "Smoothness: " << final_metrics.smoothness << std::endl;
            
            // Only revert if optimization made things significantly worse
            if (final_metrics.total_length > initial_metrics.total_length * 1.1 &&
                final_metrics.smoothness > initial_metrics.smoothness * 1.2) {
                std::cout << "Optimization degraded path quality. Reverting..." << std::endl;
                path = globalPlanner();
            }
        }
    } else {
        std::cout << "No path found after retry!\n";
    }
    return path;
}

std::vector<Node*> RRTStar::globalPlanner() {
    bool found_path = false;
    Node* final_node = nullptr;
    double goal_threshold = step_size * 1.2;  // More lenient goal threshold
    int stall_count = 0;
    // const int max_stall = 100;
    double best_distance = std::numeric_limits<double>::infinity();
    
    // Add goal node to tree occasionally
    const int goal_attempt_freq = 50;  // Try to connect to goal every 50 iterations
    const int stall_limit = 100;

    for (int i = 0; i < max_iter && !found_path; ++i) {
        Node* new_node = nullptr;
        
        // Periodically attempt to connect directly to goal
        if (i % goal_attempt_freq == 0) {
            auto goal_step = steer(nearest(goal_node.get()), goal_node.get());
            if (isCollisionFree(nearest(goal_node.get()), goal_step.get()).first) {
                new_node = goal_step.get();
                node_storage.push_back(std::move(goal_step));
            }
        }
        
        // If not connecting to goal or if that failed, get random node
        if (!new_node) {
            auto random_node = getRandomNode(i);
            Node* nearest_node = nearest(random_node.get());
            auto step_node = steer(nearest_node, random_node.get());
            
            if (isCollisionFree(nearest_node, step_node.get()).first) {
                new_node = step_node.get();
                node_storage.push_back(std::move(step_node));
            } else {
                continue;
            }
        }
        
        // Add node to tree
        new_node->parent = nearest(new_node);
        new_node->cost = new_node->parent->cost + distance(new_node->parent, new_node);
        nodes.push_back(new_node);
        
        // Update KD-tree
        kdtree->buildIndex();
        
        // Check progress
        double dist_to_goal = distance(new_node, goal_node.get());
        if (dist_to_goal < best_distance) {
            best_distance = dist_to_goal;
            stall_count = 0;
            std::cout << "New best distance: " << best_distance << std::endl;
        } else {
            stall_count++;
            if (stall_count > stall_limit) {
                std::cout << "Breaking due to lack of progress after " 
                          << stall_limit << " iterations" << std::endl;
                break;
            }
        }
        
        // Check if we reached the goal
        if (dist_to_goal < goal_threshold) {
            found_path = true;
            final_node = new_node;
            std::cout << "Goal reached! Distance: " << dist_to_goal << std::endl;
            break;
        }
        
        // Periodic progress report
        if (i % 100 == 0) {
            std::cout << "Iteration " << i << ", Best distance: " << best_distance << std::endl;
        }
        
        // Rewire nearby nodes
        auto neighbors = radiusSearch(new_node, neighbor_radius);
        rewire(neighbors, new_node);
    }
    
    std::vector<Node*> path;
    if (found_path && final_node) {
        getFinalPath(final_node, path);
        std::cout << "Path found with " << path.size() << " nodes" << std::endl;
    }
    return path;
}

void RRTStar::localPlanner(std::vector<Node*>& path) {
    // Apply path smoothing or optimization
    optimizePath(path);

    // Optionally, apply additional local refinements (e.g., gradient descent)
    refinePathDynamically(path);
}

void RRTStar::getFinalPath(Node* goal_node, std::vector<Node*>& path) {
    // Traverse the tree from the goal node to the start node
    Node* current = goal_node;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    // Reverse the path to get it in the correct order (start to goal)
    std::reverse(path.begin(), path.end());
}

void RRTStar::refinePathDynamically(std::vector<Node*>& path) {
    if (path.size() < 3) return;

    // Example: Use cubic splines for smoothing
    std::vector<std::array<double, 3>> smoothed_path;
    for (size_t i = 0; i < path.size(); ++i) {
        auto pos = RobotKinematics::computeFK(path[i]->q);
        smoothed_path.push_back(pos);
    }

    // Apply cubic spline interpolation
    applyCubicSpline(smoothed_path);

    // Update the path with smoothed configurations
    for (size_t i = 0; i < smoothed_path.size(); ++i) {
        path[i]->q = inverseKinematics(smoothed_path[i]);
    }
}

std::array<double, 6> RRTStar::inverseKinematics(const std::array<double, 3>& pos) {
    // Placeholder: Replace with actual inverse kinematics computation
    std::array<double, 6> q = {0, 0, 0, 0, 0, 0};
    // Example: Simple inverse kinematics for a 2-link planar arm
    double l1 = 1.0, l2 = 1.0;
    double r = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
    q[1] = std::acos((r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2));
    q[0] = std::atan2(pos[1], pos[0]) - std::atan2(l2 * std::sin(q[1]), l1 + l2 * std::cos(q[1]));
    return q;
}

void RRTStar::applyCubicSpline(const std::vector<std::array<double, 3>>& points) {
    size_t n = points.size();
    if (n < 2) return; // Need at least two points for interpolation

    // Step 1: Extract x, y, z coordinates from the points
    Eigen::VectorXd x(n), y(n), z(n);
    for (size_t i = 0; i < n; ++i) {
        x(i) = points[i][0];
        y(i) = points[i][1];
        z(i) = points[i][2];
    }

    // Step 2: Compute distances between consecutive points (t values)
    Eigen::VectorXd t(n);
    t(0) = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double dx = points[i][0] - points[i - 1][0];
        double dy = points[i][1] - points[i - 1][1];
        double dz = points[i][2] - points[i - 1][2];
        t(i) = t(i - 1) + std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Step 3: Construct the tridiagonal system for second derivatives
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(n);

    // Natural spline boundary conditions
    A(0, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;

    for (size_t i = 1; i < n - 1; ++i) {
        double h_i = t(i) - t(i - 1);
        double h_ip1 = t(i + 1) - t(i);

        A(i, i - 1) = h_i;
        A(i, i) = 2.0 * (h_i + h_ip1);
        A(i, i + 1) = h_ip1;

        b_x(i) = 3.0 * ((x(i + 1) - x(i)) / h_ip1 - (x(i) - x(i - 1)) / h_i);
        b_y(i) = 3.0 * ((y(i + 1) - y(i)) / h_ip1 - (y(i) - y(i - 1)) / h_i);
        b_z(i) = 3.0 * ((z(i + 1) - z(i)) / h_ip1 - (z(i) - z(i - 1)) / h_i);
    }

    // Step 4: Solve for second derivatives
    Eigen::VectorXd k_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd k_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd k_z = A.colPivHouseholderQr().solve(b_z);

    // Step 5: Generate smoothed points using cubic polynomials
    std::vector<std::array<double, 3>> smoothed_points;
    const int num_samples = 10; // Number of samples per segment

    for (size_t i = 0; i < n - 1; ++i) {
        double h = t(i + 1) - t(i);
        for (int j = 0; j <= num_samples; ++j) {
            double u = static_cast<double>(j) / num_samples;
            double a = 2.0 * u * u * u - 3.0 * u * u + 1.0;
            double b = -2.0 * u * u * u + 3.0 * u * u;
            double c = u * u * u - 2.0 * u * u + u;
            double d = u * u * u - u * u;

            double x_smooth = a * x(i) + b * x(i + 1) + h * (c * k_x(i) + d * k_x(i + 1));
            double y_smooth = a * y(i) + b * y(i + 1) + h * (c * k_y(i) + d * k_y(i + 1));
            double z_smooth = a * z(i) + b * z(i + 1) + h * (c * k_z(i) + d * k_z(i + 1));

            smoothed_points.push_back({x_smooth, y_smooth, z_smooth});
        }
    }

    // Replace the original path with the smoothed path
    // Note: You may need to convert back to joint space using inverse kinematics
    // For now, we assume the path is in Cartesian space
    std::cout << "Smoothed path generated with " << smoothed_points.size() << " points.\n";
}

std::unique_ptr<Node> RRTStar::getRandomNode(int i) {
    const double goal_bias_prob = std::min(max_goal_bias, initial_goal_bias +
        (max_goal_bias - initial_goal_bias) * (i / static_cast<double>(max_iter)));

    if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < goal_bias_prob) {
        return std::make_unique<Node>(goal_config);
    }

    std::array<double, 6> q;
    double heuristic_weight = 0.7; // Weight for heuristic bias
    for (size_t j = 0; j < 6; ++j) {
        double range = joint_limits_max[j] - joint_limits_min[j];
        double center = (joint_limits_max[j] + joint_limits_min[j]) / 2.0;
        double u = std::uniform_real_distribution<>(0.0, 1.0)(gen);
        double v = std::uniform_real_distribution<>(0.0, 1.0)(gen);

        // Bias sampling toward the goal using heuristic
        double heuristic_offset = heuristic_weight * (goal_config[j] - center);
        q[j] = center + (u < 0.5 ? 1 : -1) * range * (1.0 - std::sqrt(v)) / 2.0 + heuristic_offset;
        q[j] = std::clamp(q[j], joint_limits_min[j], joint_limits_max[j]);
    }
    return std::make_unique<Node>(q);
}

std::unique_ptr<Node> RRTStar::steer(Node* from_node, Node* to_node) {
    std::array<double, 6> new_q;
    double total_dist = 0;
    
    // Calculate direction vector
    std::array<double, 6> dq;
    for (size_t i = 0; i < 6; ++i) {
        dq[i] = to_node->q[i] - from_node->q[i];
        if (i >= 3) {  // Only normalize rotation joints
            while (dq[i] > M_PI) dq[i] -= 2 * M_PI;
            while (dq[i] < -M_PI) dq[i] += 2 * M_PI;
        }
        total_dist += dq[i] * dq[i];
    }
    total_dist = std::sqrt(total_dist);
    
    if (total_dist < 1e-6) {
        return std::make_unique<Node>(from_node->q);
    }
    
    // Adaptive step size based on distance to goal
    double dist_to_goal = distance(from_node, goal_node.get());
    double adaptive_step = step_size * (1.0 + 0.5 * std::min(1.0, dist_to_goal / neighbor_radius));
    double scale = std::min(1.0, adaptive_step / total_dist);
    
    for (size_t i = 0; i < 6; ++i) {
        new_q[i] = from_node->q[i] + dq[i] * scale;
        new_q[i] = std::clamp(new_q[i], joint_limits_min[i], joint_limits_max[i]);
    }
    
    return std::make_unique<Node>(new_q);
}

std::pair<bool, int> RRTStar::isCollisionFree(Node* node1, Node* node2) {
    if (!node1 || !node2) return {false, 0};
    
    const int steps = std::max(10, static_cast<int>(distance(node1, node2) / (step_size * 0.1)));
    
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        std::array<double, 6> q_interp;
        
        // Interpolate between configurations
        for (size_t j = 0; j < 6; j++) {
            double diff = node2->q[j] - node1->q[j];
            if (j >= 3) {  // For rotation joints
                while (diff > M_PI) diff -= 2 * M_PI;
                while (diff < -M_PI) diff += 2 * M_PI;
            }
            q_interp[j] = node1->q[j] + t * diff;
            
            if (q_interp[j] < joint_limits_min[j] - 1e-6 || 
                q_interp[j] > joint_limits_max[j] + 1e-6) {
                return {false, steps};
            }
        }
        
        // Check obstacle collision at key points
        if (i == 0 || i == steps || i % 5 == 0) {
            auto pos = RobotKinematics::computeFK(q_interp);
            if (isObstacle(pos[0], pos[1], pos[2])) {
                return {false, steps};
            }
        }
    }
    return {true, steps};
}

void RRTStar::rewire(const std::vector<Node*>& neighbors, Node* new_node) {
    for (auto* neighbor : neighbors) {
        auto [collision_free, _] = isCollisionFree(neighbor, new_node);
        if (collision_free) {
            double tentative_cost = new_node->cost + distance(new_node, neighbor);
            if (tentative_cost < neighbor->cost) {
                neighbor->parent = new_node;
                neighbor->cost = tentative_cost;
            }
        }
    }

    // Prune nodes that are no longer part of the tree
    nodes.erase(std::remove_if(nodes.begin(), nodes.end(),
                               [this](Node* node) { return node->parent == nullptr && node != start_node.get(); }),
                nodes.end());
}

double RRTStar::distance(Node* node1, Node* node2) {
    if (!node1 || !node2) return std::numeric_limits<double>::infinity();
    
    double dist = 0.0;
    // Use Euclidean distance in joint space with proper angle wrapping
    for (size_t i = 0; i < 6; i++) {
        double diff = node2->q[i] - node1->q[i];
        if (i >= 3) {  // For rotation joints
            while (diff > M_PI) diff -= 2 * M_PI;
            while (diff < -M_PI) diff += 2 * M_PI;
        }
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

Node* RRTStar::nearest(Node* target) {
    double query_pt[6];
    for (size_t i = 0; i < 6; ++i) {
        query_pt[i] = target->q[i];
    }
    unsigned int index;
    double min_dist_sq;
    kdtree->knnSearch(query_pt, 1, &index, &min_dist_sq);
    return nodes[index];
}

std::vector<Node*> RRTStar::radiusSearch(Node* target, double radius) {
    double query_pt[6];
    for (size_t i = 0; i < 6; ++i) {
        query_pt[i] = target->q[i];
    }

    // Use the correct type for ret_matches
    std::vector<nanoflann::ResultItem<unsigned int, double>> ret_matches;

    // Perform radius search
    kdtree->radiusSearch(
        query_pt,
        radius * radius,
        ret_matches,
        nanoflann::SearchParameters()
    );

    // Convert results to Node pointers
    std::vector<Node*> result;
    result.reserve(ret_matches.size());
    for (const auto& match : ret_matches) {
        result.push_back(nodes[match.first]);
    }
    return result;
}

void RRTStar::optimizePath(std::vector<Node*>& path) {
    if (path.size() <= 3) return;
    
    std::cout << "Starting path optimization. Initial size: " << path.size() << std::endl;
    
    // First pass: Try to connect nodes with larger steps
    const double max_step = step_size * 1.2;  // Allow slightly larger steps initially
    bool made_changes;
    std::vector<std::unique_ptr<Node>> temp_node_storage;
    
    do {
        made_changes = false;
        for (size_t i = 0; i < path.size() - 2; ++i) {
            double dist = distance(path[i], path[i+2]);
            if (dist <= max_step) {
                auto [collision_free, _] = isCollisionFree(path[i], path[i+2]);
                if (collision_free) {
                    path.erase(path.begin() + i + 1);
                    made_changes = true;
                    --i;  // Check the new configuration at this index
                }
            }
        }
    } while (made_changes && path.size() > 3);
    
    // Second pass: Fix any large steps
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = distance(path[i-1], path[i]);
        if (dist > step_size) {
            int num_splits = static_cast<int>(std::ceil(dist / step_size));
            std::vector<Node*> new_nodes;
            
            for (int split = 1; split < num_splits; ++split) {
                double t = static_cast<double>(split) / num_splits;
                std::array<double, 6> interpolated_q;
                
                for (size_t j = 0; j < 6; ++j) {
                    double diff = path[i]->q[j] - path[i-1]->q[j];
                    if (j >= 3) {
                        while (diff > M_PI) diff -= 2 * M_PI;
                        while (diff < -M_PI) diff += 2 * M_PI;
                    }
                    interpolated_q[j] = path[i-1]->q[j] + t * diff;
                }
                
                auto new_node = std::make_unique<Node>(interpolated_q);
                new_node->parent = path[i-1];
                new_nodes.push_back(new_node.get());
                temp_node_storage.push_back(std::move(new_node));
            }
            
            path.insert(path.begin() + i, new_nodes.begin(), new_nodes.end());
            i += new_nodes.size();
        }
    }
    
    // Move ownership of temporary nodes to permanent storage
    for (auto& node : temp_node_storage) {
        node_storage.push_back(std::move(node));
    }
    
    std::cout << "Final path size: " << path.size() << std::endl;
}

bool RRTStar::isStateValid(const std::array<double, 6>& q) {
    // Check joint limits
    for (size_t i = 0; i < 6; ++i) {
        if (q[i] < joint_limits_min[i] || q[i] > joint_limits_max[i]) {
            std::cerr << "Joint " << i << " out of bounds: " << q[i] << "\n";
            return false;
        }
    }
    
    // Compute forward kinematics
    std::array<double, 3> pos = RobotKinematics::computeFK(q);
    if (pos[0] < map_min_x || pos[0] > (map_min_x + map_width) ||
        pos[1] < map_min_y || pos[1] > (map_min_y + map_height) ||
        pos[2] < map_min_z || pos[2] > (map_min_z + map_depth)) {
        std::cerr << "Position out of bounds: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";
        return false;
    }
    return true;
}

bool RRTStar::isObstacle(double x, double y, double z) {
    std::array<double, 3> point = {x, y, z};
    for (const auto& obstacle : obstacles) {
        bool inside = true;
        for (int i = 0; i < 3; ++i) {
            if (point[i] < obstacle.min_point[i] - safety_margin ||
                point[i] > obstacle.max_point[i] + safety_margin) {
                inside = false;
                break;
            }
        }
        if (inside) return true;
    }
    return false;
}

void RRTStar::visualizePath(const std::vector<Node*>& path) {
    if (path.empty() || !visualization_enabled) return;

    // Create vectors for path
    std::vector<double> path_x, path_y, path_z;
    for (const auto& node : path) {
        auto pos = RobotKinematics::computeFK(node->q);
        path_x.push_back(pos[0]);
        path_y.push_back(pos[1]);
        path_z.push_back(pos[2]);
    }

    // Create figure
    plt::figure();

    // Plot path
    std::map<std::string, std::string> path_properties;
    path_properties["color"] = "red";
    path_properties["linestyle"] = "-";
    path_properties["label"] = "Path";
    plt::plot3(path_x, path_y, path_z, path_properties);

    // Plot all obstacles
    const double scale = 0.05;
    int obstacle_count = 0;
    for (const auto& obstacle : obstacles) {
        std::vector<double> obs_x = {obstacle.min_point[0] * scale, obstacle.max_point[0] * scale};
        std::vector<double> obs_y = {obstacle.min_point[1] * scale, obstacle.max_point[1] * scale};
        std::vector<double> obs_z = {obstacle.min_point[2] * scale, obstacle.max_point[2] * scale};

        std::map<std::string, std::string> obs_properties;
        obs_properties["color"] = "blue";
        obs_properties["linestyle"] = "--";
        obs_properties["label"] = "Obstacle " + std::to_string(++obstacle_count);
        plt::plot3(obs_x, obs_y, obs_z, obs_properties);
    }

    // Set plot properties
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::set_zlabel("Z");
    plt::title("RRT* Path Planning");
    plt::legend();
    plt::show();
}

PathQualityMetrics RRTStar::evaluatePathQuality(const std::vector<Node*>& path) {
    PathQualityMetrics metrics;
    if (path.size() < 2) return metrics;
    
    std::vector<double> step_sizes;
    for (size_t i = 1; i < path.size(); ++i) {
        double step = distance(path[i-1], path[i]);
        metrics.total_length += step;
        metrics.max_step = std::max(metrics.max_step, step);
        step_sizes.push_back(step);
    }
    
    metrics.avg_step = metrics.total_length / (path.size() - 1);
    
    // Calculate path smoothness as variance in step sizes
    double variance = 0.0;
    for (double step : step_sizes) {
        variance += std::pow(step - metrics.avg_step, 2);
    }
    metrics.smoothness = step_sizes.empty() ? 0.0 : std::sqrt(variance / step_sizes.size());
    
    return metrics;
}
