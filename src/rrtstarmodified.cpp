#include "rrtstarmodified.h"
#include <iostream>
#include <algorithm>
#include <cmath>
#include "matplotlibcpp.h"
#include <map>
#include <stdexcept>
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

// REMOVE: duplicate constants - these are already in header
// constexpr double LINK_LENGTHS[] = {0.0892, 0.425, 0.392, 0.1093, 0.09475, 0.0825};
// constexpr double MAX_JOINT_VEL = M_PI/4;
// constexpr double CONTROL_RATE = 100.0;

namespace RobotKinematics {
    // [NEW] Proper 6-DOF forward kinematics
    Eigen::Isometry3d computeFK(const std::array<double, 6>& q) {
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        
        // DH parameters (a, d, alpha, theta)
        const double dh[6][4] = {
            {0, LINK_LENGTHS[0], M_PI/2, q[0]},   // Joint 1
            {LINK_LENGTHS[1], 0, 0, q[1]},        // Joint 2
            {LINK_LENGTHS[2], 0, 0, q[2]},        // Joint 3
            {0, LINK_LENGTHS[3], M_PI/2, q[3]},   // Joint 4
            {0, LINK_LENGTHS[4], -M_PI/2, q[4]},  // Joint 5
            {0, LINK_LENGTHS[5], 0, q[5]}         // Joint 6
        };

        for(int i = 0; i < 6; ++i) {
            double ct = cos(dh[i][3]);
            double st = sin(dh[i][3]);
            double ca = cos(dh[i][2]);
            double sa = sin(dh[i][2]);

            Eigen::Matrix4d A;
            A << ct, -st*ca,  st*sa, dh[i][0]*ct,
                 st,  ct*ca, -ct*sa, dh[i][0]*st,
                 0,    sa,     ca,    dh[i][1],
                 0,    0,      0,     1;
            
            T = T * Eigen::Isometry3d(A);
        }
        return T;
    }

    // [NEW] Numerical inverse kinematics using Newton-Raphson
    // FIXED: Remove default parameters in implementation
    std::array<double, 6> inverseKinematics(const Eigen::Vector3d& target_pos,
                                            const std::array<double, 6>& q_init,
                                            double tol,
                                            int max_iter) {
        std::array<double, 6> q = q_init;
        Eigen::MatrixXd J(3, 6);
        Eigen::Vector3d error;
        
        for(int iter = 0; iter < max_iter; ++iter) {
            Eigen::Isometry3d T = computeFK(q);
            error = target_pos - T.translation();
            
            if(error.norm() < tol) break;

            // Compute Jacobian numerically
            const double dq = 1e-6;
            for(int i = 0; i < 6; ++i) {
                std::array<double, 6> q_plus = q;
                q_plus[i] += dq;
                Eigen::Vector3d pos_plus = computeFK(q_plus).translation();
                
                std::array<double, 6> q_minus = q;
                q_minus[i] -= dq;
                Eigen::Vector3d pos_minus = computeFK(q_minus).translation();
                
                J.col(i) = (pos_plus - pos_minus)/(2*dq);
            }

            // Damped least squares
            Eigen::MatrixXd Jinv = (J.transpose() * J + 0.001*Eigen::MatrixXd::Identity(6,6))
                                  .inverse() * J.transpose();
            Eigen::VectorXd dq_vec = Jinv * error;
            
            for(int i = 0; i < 6; ++i) {
                q[i] += dq_vec(i);
                // Angle wrapping
                if(i >= 3) {
                    q[i] = fmod(q[i] + M_PI, 2*M_PI) - M_PI;
                }
            }
        }
        return q;
    }
}

RRTStarModified::RRTStarModified(const std::array<double, 6>& start_q, const std::array<double, 6>& goal_q,
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
    //   current_joints(start_q),
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

RRTStarModified::~RRTStarModified() = default;

bool RRTStarModified::lineAABBIntersection(const std::array<double, 3>& start,
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
std::vector<PlanningObstacle> RRTStarModified::obstacles = {
    PlanningObstacle({2, 2, 2}, {4, 4, 4}),  // Near the start
    PlanningObstacle({6, 6, 6}, {8, 8, 8})   // Near the goal
};

std::vector<Node*> RRTStarModified::globalPlanner() {
    bool found_path = false;
    Node* final_node = nullptr;
    double goal_threshold = step_size * 1.2;  // More lenient goal threshold
    int stall_count = 0;
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

void RRTStarModified::localPlanner(std::vector<Node*>& path) {
    // Apply path smoothing or optimization
    optimizePath(path);

    // Optionally, apply additional local refinements (e.g., gradient descent)
    refinePathDynamically(path);
}

void RRTStarModified::getFinalPath(Node* goal_node, std::vector<Node*>& path) {
    // Traverse the tree from the goal node to the start node
    Node* current = goal_node;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    // Reverse the path to get it in the correct order (start to goal)
    std::reverse(path.begin(), path.end());
}

void RRTStarModified::refinePathDynamically(std::vector<Node*>& path) {
    if (path.size() < 3) return;

    // Example: Use cubic splines for smoothing
    std::vector<std::array<double, 3>> smoothed_path;
    for (size_t i = 0; i < path.size(); ++i) {
        auto T = RobotKinematics::computeFK(path[i]->q);
        Eigen::Vector3d pos = T.translation();
        smoothed_path.push_back({pos.x(), pos.y(), pos.z()});
    }

    // Apply cubic spline interpolation
    applyCubicSpline(smoothed_path);

    // Update the path with smoothed configurations
    for (size_t i = 0; i < smoothed_path.size(); ++i) {
        path[i]->q = inverseKinematics(smoothed_path[i]);
    }
}

std::array<double, 6> RRTStarModified::inverseKinematics(const std::array<double, 3>& pos) {
    // Placeholder: Replace with actual inverse kinematics computation
    std::array<double, 6> q = {0, 0, 0, 0, 0, 0};
    // Example: Simple inverse kinematics for a 2-link planar arm
    double l1 = 1.0, l2 = 1.0;
    double r = std::sqrt(pos[0] * pos[0] + pos[1] * pos[1]);
    q[1] = std::acos((r * r - l1 * l1 - l2 * l2) / (2 * l1 * l2));
    q[0] = std::atan2(pos[1], pos[0]) - std::atan2(l2 * std::sin(q[1]), l1 + l2 * std::cos(q[1]));
    return q;
}

void RRTStarModified::applyCubicSpline(const std::vector<std::array<double, 3>>& points) {
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

std::unique_ptr<Node> RRTStarModified::getRandomNode(int i) {
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

std::unique_ptr<Node> RRTStarModified::steer(Node* from_node, Node* to_node) {
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

double RRTStarModified::distance(Node* node1, Node* node2) {
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

Node* RRTStarModified::nearest(Node* target) {
    double query_pt[6];
    for (size_t i = 0; i < 6; ++i) {
        query_pt[i] = target->q[i];
    }
    unsigned int index;
    double min_dist_sq;
    kdtree->knnSearch(query_pt, 1, &index, &min_dist_sq);
    return nodes[index];
}

std::vector<Node*> RRTStarModified::radiusSearch(Node* target, double radius) {
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

bool RRTStarModified::isStateValid(const std::array<double, 6>& q) {
    // Check joint limits
    for (size_t i = 0; i < 6; ++i) {
        if (q[i] < joint_limits_min[i] || q[i] > joint_limits_max[i]) {
            std::cerr << "Joint " << i << " out of bounds: " << q[i] << "\n";
            return false;
        }
    }
    
    // Compute forward kinematics
    auto T = RobotKinematics::computeFK(q);
    Eigen::Vector3d pos = T.translation();
    if (pos.x() < map_min_x || pos.x() > (map_min_x + map_width) ||
        pos.y() < map_min_y || pos.y() > (map_min_y + map_height) ||
        pos.z() < map_min_z || pos.z() > (map_min_z + map_depth)) {
        std::cerr << "Position out of bounds: (" << pos.x() << ", " << pos.y() << ", " << pos.z() << ")\n";
        return false;
    }
    return true;
}

bool RRTStarModified::isObstacle(double x, double y, double z) {
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

void RRTStarModified::visualizePath(const std::vector<Node*>& path) {
    if (path.empty() || !visualization_enabled) return;

    // Create vectors for path
    std::vector<double> path_x, path_y, path_z;
    for (const auto& node : path) {
        auto T = RobotKinematics::computeFK(node->q);
        Eigen::Vector3d pos = T.translation();
        path_x.push_back(pos.x());
        path_y.push_back(pos.y());
        path_z.push_back(pos.z());
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

PathQualityMetrics RRTStarModified::evaluatePathQuality(const std::vector<Node*>& path) {
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

// [NEW] Quintic spline interpolation in joint space
void RRTStarModified::applyQuinticSplineWithConstraints(std::vector<Node*>& path) {
    if(path.size() < 4) return;

    const int n = path.size();
    Eigen::MatrixXd q(6, n);
    Eigen::VectorXd time(n);
    
    // Populate joint angles and timestamps
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j < 6; ++j) {
            q(j, i) = path[i]->q[j];
        }
        time(i) = i;
    }
    
    // Compute time scaling to respect velocity limits
    for (int i = 1; i < n; ++i) {
        Eigen::VectorXd dq = q.col(i) - q.col(i-1);
        
        // Find the joint that takes the longest time due to velocity limit
        double max_time_vel = 0.0;
        for (int j = 0; j < 6; ++j) {
            double diff = dq(j);
            if (j >= 3) {
                // Normalize angles for rotational joints
                while(diff > M_PI) diff -= 2*M_PI;
                while(diff < -M_PI) diff += 2*M_PI;
            }
            double joint_time = std::abs(diff) / MAX_JOINT_VEL;
            max_time_vel = std::max(max_time_vel, joint_time);
        }
        
        // Update timestamp to respect velocity constraint (minimum of 0.1s between waypoints)
        time(i) = time(i-1) + std::max(0.1, max_time_vel);
    }

    // Quintic spline interpolation with velocity and acceleration constraints
    std::vector<Node*> smoothed_path;
    const double dt = 1.0 / CONTROL_RATE; // Control rate in seconds
    
    for(double t = time(0); t < time(n-1); t += dt) {
        // Find the segment this time belongs to
        int idx = std::upper_bound(time.data(), time.data()+n, t) - time.data();
        if (idx <= 1) idx = 1;
        if (idx >= n) idx = n-1;
        
        // Local time parameter [0,1] within the segment
        double local_t = (t - time(idx-1)) / (time(idx) - time(idx-1));
        local_t = std::max(0.0, std::min(1.0, local_t)); // Clamp to [0,1]
        
        // Quintic polynomial basis
        double t2 = local_t * local_t;
        double t3 = t2 * local_t;
        double t4 = t3 * local_t;
        double t5 = t4 * local_t;
        
        std::array<double, 6> q_new;
        std::array<double, 6> dq_new; // Velocity (for constraint checking)
        
        for(int j = 0; j < 6; ++j) {
            double q0 = q(j, idx-1);
            double q1 = q(j, idx);
            
            // Boundary conditions for smooth velocity and acceleration
            double dq0 = (idx > 1) ? (q(j, idx) - q(j, idx-2)) / (time(idx) - time(idx-2)) : 0.0;
            double dq1 = (idx < n-1) ? (q(j, idx+1) - q(j, idx-1)) / (time(idx+1) - time(idx-1)) : 0.0;
            double ddq0 = 0.0; // Zero acceleration at endpoints for smoothness
            double ddq1 = 0.0;
            
            // Limit velocities to MAX_JOINT_VEL
            dq0 = std::min(MAX_JOINT_VEL, std::max(-MAX_JOINT_VEL, dq0));
            dq1 = std::min(MAX_JOINT_VEL, std::max(-MAX_JOINT_VEL, dq1));
            
            // Compute quintic polynomial coefficients
            double a0 = q0;
            double a1 = dq0;
            double a2 = 0.5 * ddq0;
            double a3 = 10.0 * (q1 - q0) - 6.0 * dq0 - 4.0 * dq1 - 1.5 * ddq0 + 0.5 * ddq1;
            double a4 = 15.0 * (q0 - q1) + 8.0 * dq0 + 7.0 * dq1 + 1.5 * ddq0 - ddq1;
            double a5 = 6.0 * (q1 - q0) - 3.0 * (dq0 + dq1) - 0.5 * (ddq0 - ddq1);
            
            // Evaluate position and velocity
            q_new[j] = a0 + a1*local_t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
            dq_new[j] = a1 + 2.0*a2*local_t + 3.0*a3*t2 + 4.0*a4*t3 + 5.0*a5*t4;
            
            // Velocity constraint enforcement
            double vel_scale = 1.0;
            if (std::abs(dq_new[j]) > MAX_JOINT_VEL) {
                vel_scale = std::min(vel_scale, MAX_JOINT_VEL / std::abs(dq_new[j]));
            }
            
            // Apply velocity scaling if needed
            if (vel_scale < 1.0) {
                // Adjust position based on scaled velocity
                // This is a simplified approach; a better method would replan the entire trajectory
                dq_new[j] *= vel_scale;
                q_new[j] = q0 + dq_new[j] * ((time(idx) - time(idx-1)) * local_t);
            }
        }
        
        auto* node = new Node(q_new);
        smoothed_path.push_back(node);
    }

    // Add the final node if not already there
    if (!smoothed_path.empty()) {
        double dist_to_goal = distance(smoothed_path.back(), path.back());
        if (dist_to_goal > 1e-6) {
            smoothed_path.push_back(new Node(path.back()->q));
        }
    }

    // Cleanup and replace path
    for(auto* node : path) delete node;
    path = smoothed_path;
}

void RRTStarModified::refinePathWithIK(std::vector<Node*>& path) {
    std::vector<std::array<double, 3>> cart_path;
    for(auto* node : path) {
        auto T = RobotKinematics::computeFK(node->q);
        Eigen::Vector3d pos = T.translation();
        cart_path.push_back({pos.x(), pos.y(), pos.z()});
    }
    
    applyCubicSpline(cart_path);
    
    std::vector<Node*> new_path;
    for(const auto& pt : cart_path) {
        Eigen::Vector3d target(pt[0], pt[1], pt[2]);
        auto q = RobotKinematics::inverseKinematics(target, 
            new_path.empty() ? path[0]->q : new_path.back()->q);
        new_path.push_back(new Node(q));
    }
    
    // Replace old path
    for(auto* node : path) delete node;
    path = new_path;
}

// [NEW] Velocity-constrained path optimization
void RRTStarModified::optimizePath(std::vector<Node*>& path) {
    if(path.size() < 3) return;

    // Phase 1: Shortcutting
    bool changed;
    do {
        changed = false;
        for(size_t i = 0; i < path.size() - 2; ++i) {
            double dist = distance(path[i], path[i+2]);
            if(dist <= step_size * 1.2) {
                auto [free, steps] = isCollisionFree(path[i], path[i+2]);
                if(free) {
                    // Only delete if node is not managed by node_storage
                    bool is_managed = false;
                    for (const auto& ptr : node_storage) {
                        if (ptr.get() == path[i+1]) {
                            is_managed = true;
                            break;
                        }
                    }
                    if (!is_managed) {
                        delete path[i+1];
                    }
                    path.erase(path.begin() + i + 1);
                    changed = true;
                    --i;
                }
            }
        }
    } while(changed && path.size() > 3);

    // Phase 2: Velocity-constrained interpolation
    std::vector<Node*> new_path;
    for(size_t i = 1; i < path.size(); ++i) {
        Node* prev_node = path[i-1];
        Node* curr_node = path[i];
        
        new_path.push_back(prev_node);
        
        // Joint space distance calculation
        double joint_dist = distance(prev_node, curr_node);
        double dt = joint_dist / MAX_JOINT_VEL;
        int num_steps = std::max(1, static_cast<int>(std::ceil(dt * CONTROL_RATE)));
        
        // Joint space interpolation
        for(int j = 1; j < num_steps; ++j) {
            double t = static_cast<double>(j)/num_steps;
            std::array<double, 6> q;
            
            for(int k = 0; k < 6; ++k) {
                double diff = curr_node->q[k] - prev_node->q[k];
                // Handle angular wrapping for rotational joints
                if(k >= 3) {
                    while(diff > M_PI) diff -= 2*M_PI;
                    while(diff < -M_PI) diff += 2*M_PI;
                }
                q[k] = prev_node->q[k] + t * diff;
                q[k] = std::clamp(q[k], joint_limits_min[k], joint_limits_max[k]);
            }
            
            // Create new node with proper parent relationship
            auto* new_node = new Node(q);
            new_node->parent = prev_node;
            node_storage.push_back(std::unique_ptr<Node>(new_node)); // Transfer ownership
            new_path.push_back(new_node);
        }
    }
    new_path.push_back(path.back());

    // Preserve original node ownership
    path = new_path;
}

void RRTStarModified::rewire(const std::vector<Node*>& neighbors, Node* new_node) {
    for(auto* neighbor : neighbors) {
        auto [collision_free, steps] = isCollisionFree(neighbor, new_node);
        if(collision_free) {
            // Base cost = distance
            double dist_cost = distance(neighbor, new_node);
            
            // Angular change penalty
            double angular_cost = 0;
            for(int i = 0; i < 6; ++i) {
                double diff = new_node->q[i] - neighbor->q[i];
                if(i >= 3) {
                    while(diff > M_PI) diff -= 2*M_PI;
                    while(diff < -M_PI) diff += 2*M_PI;
                }
                angular_cost += std::abs(diff);
            }
            
            // Angular velocity consistency penalty (jerk minimization)
            double jerk_cost = 0.0;
            if (neighbor->parent != nullptr) {
                std::array<double, 6> prev_velocity, curr_velocity;
                
                // Compute previous angular velocity
                for (int i = 0; i < 6; ++i) {
                    double diff = neighbor->q[i] - neighbor->parent->q[i];
                    if (i >= 3) {
                        while(diff > M_PI) diff -= 2*M_PI;
                        while(diff < -M_PI) diff += 2*M_PI;
                    }
                    prev_velocity[i] = diff;
                }
                
                // Compute current angular velocity
                for (int i = 0; i < 6; ++i) {
                    double diff = new_node->q[i] - neighbor->q[i];
                    if (i >= 3) {
                        while(diff > M_PI) diff -= 2*M_PI;
                        while(diff < -M_PI) diff += 2*M_PI;
                    }
                    curr_velocity[i] = diff;
                }
                
                // Compute change in angular velocity (jerk)
                for (int i = 0; i < 6; ++i) {
                    double velocity_diff = curr_velocity[i] - prev_velocity[i];
                    jerk_cost += velocity_diff * velocity_diff;
                }
                
                jerk_cost = std::sqrt(jerk_cost);
            }
            
            // Combine costs with appropriate weights
            double total_cost = neighbor->cost + 
                                0.5 * dist_cost +       // Distance weight
                                0.3 * angular_cost +    // Angular change weight
                                0.2 * jerk_cost;        // Jerk minimization weight
            
            if(total_cost < new_node->cost) {
                new_node->parent = neighbor;
                new_node->cost = total_cost;
            }
        }
    }
}

// [NEW] Enhanced collision checking with joint limits
std::pair<bool, int> RRTStarModified::isCollisionFree(Node* node1, Node* node2) {
    const int steps = std::max(10, static_cast<int>(distance(node1, node2)/0.1));
    
    for(int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i)/steps;
        std::array<double, 6> q;
        for(int j = 0; j < 6; ++j) {
            q[j] = node1->q[j] + t*(node2->q[j] - node1->q[j]);
            
            // Strict joint limit checking
            if(q[j] < joint_limits_min[j] - 1e-6 || 
               q[j] > joint_limits_max[j] + 1e-6) {
                return {false, steps};
            }
        }
        
        // Forward kinematics collision check
        auto T = RobotKinematics::computeFK(q);
        Eigen::Vector3d pos = T.translation();
        if(isObstacle(pos.x(), pos.y(), pos.z())) {
            return {false, steps};
        }
    }
    return {true, steps};
}

// [NEW] Main planning sequence with full smoothing
std::vector<Node*> RRTStarModified::findPath() {
    // Phase 0: Global planning
    auto path = globalPlanner();
    if(path.empty()) {
        std::cout << "Initial planning failed, retrying..." << std::endl;
        path = globalPlanner();
        if(path.empty()) return path;
    }

    std::cout << "Initial path found with " << path.size() << " nodes" << std::endl;
    
    // Save initial path metrics
    auto initial_metrics = evaluatePathQuality(path);
    std::cout << "Initial path metrics:" << std::endl;
    std::cout << "Total length: " << initial_metrics.total_length << std::endl;
    std::cout << "Max step: " << initial_metrics.max_step << std::endl;
    std::cout << "Average step: " << initial_metrics.avg_step << std::endl;
    std::cout << "Smoothness: " << initial_metrics.smoothness << std::endl;
    
    // Phase 1: Basic path optimization (shortcutting)
    std::cout << "Applying basic path optimization..." << std::endl;
    optimizePath(path);
    
    // Phase 2: Cartesian smoothing with accurate IK
    if(!path.empty() && path.size() > 2)
    {
        std::cout << "Refining path with IK..." << std::endl;
        refinePathWithIK(path);
    }
    
    // Phase 3: Joint-space smoothing with velocity constraints
    if(!path.empty() && path.size() > 3) 
    {
        std::cout << "Applying velocity-constrained quintic spline..." << std::endl;
        applyQuinticSplineWithConstraints(path);
    }
    
    // Phase 4: Final optimization for velocity consistency
    if(!path.empty() && path.size() > 2) 
    {
        std::cout << "Performing final velocity-based optimization..." << std::endl;
        optimizePath(path);
    }
    
    // Output final path metrics
    if(!path.empty()) {
        auto final_metrics = evaluatePathQuality(path);
        std::cout << "\nFinal path metrics:" << std::endl;
        std::cout << "Path nodes: " << path.size() << std::endl;
        std::cout << "Total length: " << final_metrics.total_length << std::endl;
        std::cout << "Max step: " << final_metrics.max_step << std::endl;
        std::cout << "Average step: " << final_metrics.avg_step << std::endl;
        std::cout << "Smoothness: " << final_metrics.smoothness << std::endl;
        
        // Verify all path steps meet velocity constraints
        bool velocity_constraints_met = true;
        for(size_t i = 1; i < path.size(); ++i) 
        {
            double step_size = distance(path[i-1], path[i]);
            if(step_size > MAX_JOINT_VEL/CONTROL_RATE * 1.1) 
            {
                std::cout << "Warning: Velocity constraint violated at step " << i 
                          << ": " << step_size << " > " 
                          << MAX_JOINT_VEL/CONTROL_RATE << std::endl;
                velocity_constraints_met = false;
            }
        }
        
        if(velocity_constraints_met) 
        {
            std::cout << "All velocity constraints satisfied!" << std::endl;
        }
    }

    return path;
}