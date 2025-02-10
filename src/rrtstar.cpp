#include "rrtstar.h"
// #include "spline.h"
#include <cmath>
#include <algorithm>
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

// std::array<DHParameters, 6> RobotKinematics::dh_params = {
//     DHParameters(0,      M_PI / 2,  50,   0),  // Base to shoulder, Base (d1)
//     DHParameters(40,     0,         0,    0),  // Shoulder to elbow, Shoulder (a1)
//     DHParameters(40,     0,         0,    0),  // Elbow to wrist, Elbow (a2)
//     DHParameters(0,      M_PI / 2,  20,   0),  // Wrist 1, Wrist 1 (d3)
//     DHParameters(0,     -M_PI / 2,  0,    0),  // Wrist 2, Wrist 2 (d5)
//     DHParameters(0,      0,        15,    0)   // End effector, End effector (d6)
// };

std::array<DHParameters, 6> RobotKinematics::dh_params = {
    // RB5-850 parameters (a, alpha, d, theta)
    DHParameters(0,      M_PI/2,  169.2, 0),  // Base (d1)
    DHParameters(425.0,  0,       0,     0),  // Shoulder (a1)
    DHParameters(392.0,  0,       0,     0),  // Elbow (a2)
    DHParameters(0,      M_PI/2,  148.4, 0),  // Wrist 1 (d3)
    DHParameters(0,     -M_PI/2,  110.7, 0),  // Wrist 2 (d5)
    DHParameters(0,      0,       96.7,  0)   // End effector (d6)
};

// Define 3D obstacles with AABB (Axis-Aligned Bounding Box)
// std::vector<Obstacle> RRTStar::obstacles = {
//     Obstacle({45, 45, 45}, {55, 55, 55}),  // First cubic obstacle
//     Obstacle({65, 65, 65}, {75, 75, 75})   // Second cubic obstacle
// };

// In rrtstar.cpp
std::vector<Obstacle> RRTStar::obstacles = {
    // Avoid placing obstacles near the goal (300, 300, 500)
    Obstacle({200, 200, 200}, {250, 250, 250}),  // Example obstacle away from goal
    // Obstacle({-400, -400, 100}, {-300, -300, 300})
};

// RRTStar::RRTStar(const std::array<double, 6>& start_q, const std::array<double, 6>& goal_q,
//                  double map_width, double map_height, double map_depth,
//                  double step_size, double neighbor_radius,
//                  double safety_margin, int max_iter,
//                  double min_x, double min_y, double min_z)
//     : map_width(map_width), map_height(map_height), map_depth(map_depth),
//       map_min_x(min_x), map_min_y(min_y), map_min_z(min_z),
//       step_size(step_size), neighbor_radius(neighbor_radius),
//       safety_margin(safety_margin), max_iter(max_iter),
//       gen(std::random_device{}()),
//       dis_x(min_x, min_x + map_width), 
//       dis_y(min_y, min_y + map_height), 
//       dis_z(min_z, min_z + map_depth),
//       goal_config(goal_q), // Initialize goal_config first
//       node_adapter(nodes)  // Initialize node_adapter second
// {
    
//     std::cout << "Workspace bounds:\n"
//               << "X: [" << map_min_x << ", " << map_min_x + map_width << "]\n"
//               << "Y: [" << map_min_y << ", " << map_min_y + map_height << "]\n"
//               << "Z: [" << map_min_z << ", " << map_min_z + map_depth << "]\n";
              
//     if (!isStateValid(start_q) || !isStateValid(goal_q)) {
//         throw std::invalid_argument("Start or goal configuration is invalid!");
//     }
//     start_node = std::make_unique<Node>(start_q);
//     goal_node = std::make_unique<Node>(goal_q);
//     nodes.push_back(start_node.get());

//     // Fixed the template syntax by putting it all on one line
//     kdtree = std::make_unique<nanoflann::KDTreeSingleIndexAdaptor<nanoflann::L2_Simple_Adaptor<double, NodeAdapter>, NodeAdapter, 6, unsigned int>>(
//         6, node_adapter, nanoflann::KDTreeSingleIndexAdaptorParams());
//     kdtree->buildIndex();
// }

std::array<double, 6> RRTStar::cartesianToJointSpace(
    double x, double y, double z, double rx, double ry, double rz) {
    // Get multiple IK solutions
    auto solutions = ik_solver.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);
    if (solutions.empty()) {
        throw std::runtime_error("No valid IK solution found!");
    }

    // Convert degrees to radians and find best solution
    double best_cost = std::numeric_limits<double>::infinity();
    std::array<double, 6> best_solution;
    bool found_valid = false;

    for (const auto& sol : solutions) {
        std::array<double, 6> config;
        bool valid = true;
        double cost = 0;

        // Convert to radians and check joint limits
        for (size_t i = 0; i < 6; i++) {
            config[i] = normalizeAngle(deg2rad(sol.joints[i]));
            
            // Check if within joint limits
            if (config[i] < joint_limits_min[i] || config[i] > joint_limits_max[i]) {
                valid = false;
                break;
            }
            
            // Add to cost (prefer solutions closer to current position)
            cost += config[i] * config[i];  // Simple Euclidean distance in joint space
        }

        if (valid && cost < best_cost) {
            best_cost = cost;
            best_solution = config;
            found_valid = true;
        }
    }

    if (!found_valid) {
        std::cout << "No solution found within joint limits!" << std::endl;
        std::cout << "Available solutions:" << std::endl;
        for (const auto& sol : solutions) {
            std::cout << "Configuration: " << sol.configuration.shoulder 
                     << ", " << sol.configuration.elbow 
                     << ", " << sol.configuration.wrist << std::endl;
            std::cout << "Joint angles (deg): ";
            for (double angle : sol.joints) {
                std::cout << angle << " ";
            }
            std::cout << std::endl;
        }
        throw std::runtime_error("No valid IK solution within joint limits!");
    }

    return best_solution;
}

// Helper function to normalize angle to [-π, π]
double RRTStar::normalizeAngle(double angle) {
    while (angle > M_PI) angle -= 2 * M_PI;
    while (angle < -M_PI) angle += 2 * M_PI;
    return angle;
}

// Helper function to convert degrees to radians
double RRTStar::deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

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
      dis_z(min_z, min_z + map_depth), // Initialize dis_z before goal_config
      goal_config(goal_q),            // Initialize goal_config after dis_z
      node_adapter(nodes)             // Initialize node_adapter last
{
    std::cout << "Workspace bounds:\n"
              << "X: [" << map_min_x << ", " << map_min_x + map_width << "]\n"
              << "Y: [" << map_min_y << ", " << map_min_y + map_height << "]\n"
              << "Z: [" << map_min_z << ", " << map_min_z + map_depth << "]\n";
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
    std::array<double, 3> dir = {
        (end[0] - start[0]) * 0.5,
        (end[1] - start[1]) * 0.5,
        (end[2] - start[2]) * 0.5
    };
    
    std::array<double, 3> mid = {
        (start[0] + end[0]) * 0.5,
        (start[1] + end[1]) * 0.5,
        (start[2] + end[2]) * 0.5
    };
    
    std::array<double, 3> extent = {
        std::abs(dir[0]), std::abs(dir[1]), std::abs(dir[2])
    };
    
    // Normalize direction vector
    double length = std::sqrt(dir[0]*dir[0] + dir[1]*dir[1] + dir[2]*dir[2]);
    if (length > 1e-9) {
        for (int i = 0; i < 3; i++) dir[i] /= length;
    }

    std::array<double, 3> box_center = {
        (box_min[0] + box_max[0]) * 0.5,
        (box_min[1] + box_max[1]) * 0.5,
        (box_min[2] + box_max[2]) * 0.5
    };
    
    std::array<double, 3> box_half = {
        (box_max[0] - box_min[0]) * 0.5,
        (box_max[1] - box_min[1]) * 0.5,
        (box_max[2] - box_min[2]) * 0.5
    };

    std::array<double, 3> t = {
        mid[0] - box_center[0],
        mid[1] - box_center[1],
        mid[2] - box_center[2]
    };

    // Test axes
    for (int i = 0; i < 3; i++) {
        if (std::abs(t[i]) > box_half[i] + extent[i]) return false;
    }

    // Test cross products
    for (int i = 0; i < 3; i++) {
        int a = (i+1)%3;
        int b = (i+2)%3;
        double radius = box_half[a] * std::abs(dir[b]) + box_half[b] * std::abs(dir[a]);
        double distance = std::abs(t[a] * dir[b] - t[b] * dir[a]);
        if (distance > radius) return false;
    }

    return true;
}

// std::array<double, 3> RobotKinematics::computeFKPosition(const std::array<double, 6>& q) {
//     // Example FK: Simple spherical coordinates
//     double x = 100 * std::cos(q[0]) * std::cos(q[1]);
//     double y = 100 * std::sin(q[0]) * std::cos(q[1]);
//     double z = 100 * std::sin(q[1]);
//     return {x, y, z};
// }

std::vector<Node*> RRTStar::findPath() {
    for (int i = 0; i < max_iter; ++i) {
        auto random_node = getRandomNode();
        Node* nearest_node = nearest(random_node.get());
        auto new_node = steer(nearest_node, random_node.get());
        if (isCollisionFree(nearest_node, new_node.get())) {
            std::vector<Node*> neighbors = radiusSearch(new_node.get(), neighbor_radius);
            Node* min_cost_node = nearest_node;
            double min_cost = nearest_node->cost + distance(nearest_node, new_node.get());
            for (auto& neighbor : neighbors) {
                if (isCollisionFree(neighbor, new_node.get())) {
                    double tentative_cost = neighbor->cost + distance(neighbor, new_node.get());
                    if (tentative_cost < min_cost) {
                        min_cost = tentative_cost;
                        min_cost_node = neighbor;
                    }
                }
            }
            new_node->parent = min_cost_node;
            new_node->cost = min_cost;
            nodes.push_back(new_node.get());
            node_storage.push_back(std::move(new_node));
            kdtree->buildIndex();
            // Rebuild KD-tree periodically
            if (nodes.size() % 100 == 0) {
                kdtree->buildIndex();
            }
            rewire(neighbors, nodes.back());
            if (distance(nodes.back(), goal_node.get()) <= step_size &&
                isCollisionFree(nodes.back(), goal_node.get())) {
                goal_node->parent = nodes.back();
                goal_node->cost = nodes.back()->cost + distance(nodes.back(), goal_node.get());
                break;
            }
        }
    }
    std::vector<Node*> path;
    if (goal_node->parent != nullptr) {
        getFinalPath(goal_node.get(), path);
    }
    return path;
}

std::unique_ptr<Node> RRTStar::getRandomNode() {
    const double goal_bias_prob = 0.1; // 10% chance to sample goal
    if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < goal_bias_prob) {
        return std::make_unique<Node>(goal_config); // Use the goal configuration
    }
    std::array<double, 6> q;
    int max_attempts = 100;
    int attempts = 0;
    do {
        for (size_t i = 0; i < 6; ++i) {
            double range = joint_limits_max[i] - joint_limits_min[i];
            double center = (joint_limits_max[i] + joint_limits_min[i]) / 2.0;
            double u = std::uniform_real_distribution<>(0.0, 1.0)(gen);
            double v = std::uniform_real_distribution<>(0.0, 1.0)(gen);
            q[i] = center + (u < 0.5 ? 1 : -1) * range * (1.0 - std::sqrt(v)) / 2.0;
        }
        attempts++;
        if (attempts >= max_attempts) {
            std::cout << "Maximum attempts reached in getRandomNode" << std::endl;
            // Try midpoint of joint limits
            for (size_t i = 0; i < 6; i++) {
                q[i] = (joint_limits_max[i] + joint_limits_min[i]) / 2.0;
            }
            break;
        }
        auto pos = RobotKinematics::computeFK(q);

        // Debugging output
        std::cout << "Joint angles: ";
        for (double angle : q) std::cout << angle << " ";
        std::cout << "\nEnd-effector position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

        if (attempts % 10 == 0) {
            std::cout << "Attempt " << attempts << " - Position: "
                      << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;
        }
    } while (!isStateValid(q));
    return std::make_unique<Node>(q);
}

// std::unique_ptr<Node> RRTStar::steer(Node* nearest_node, Node* random_node) {
//     std::array<double, 6> new_q;
//     for (size_t i = 0; i < 6; ++i) {
//         double dq = random_node->q[i] - nearest_node->q[i];
//         new_q[i] = nearest_node->q[i] + dq * step_size / distance(nearest_node, random_node);
//         new_q[i] = std::max(joint_limits_min[i], std::min(new_q[i], joint_limits_max[i]));
//     }

//     // Debugging output
//     auto pos = RobotKinematics::computeFK(new_q);
//     std::cout << "Steered joint angles: ";
//     for (double angle : new_q) std::cout << angle << " ";
//     std::cout << "\nSteered end-effector position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

//     return std::make_unique<Node>(new_q);
// }

std::unique_ptr<Node> RRTStar::steer(Node* nearest_node, Node* random_node) {
    std::array<double, 6> new_q;
    for (size_t i = 0; i < 6; ++i) {
        double dq = random_node->q[i] - nearest_node->q[i];
        // Handle angular wrap-around
        if (dq > M_PI) dq -= 2 * M_PI;
        if (dq < -M_PI) dq += 2 * M_PI;
        new_q[i] = nearest_node->q[i] + dq * step_size / distance(nearest_node, random_node);
        // Enforce joint limits
        new_q[i] = std::clamp(new_q[i], joint_limits_min[i], joint_limits_max[i]);
    }
    return std::make_unique<Node>(new_q);
}

bool RRTStar::isCollisionFree(Node* node1, Node* node2) {
    const int steps = 10;  // Number of interpolation steps
    for (int i = 0; i <= steps; i++) {
        double t = static_cast<double>(i) / steps;
        std::array<double, 6> q_interp;
        // Interpolate joint values
        for (size_t j = 0; j < 6; j++) {
            double diff = node2->q[j] - node1->q[j];
            // Handle angular wrap-around
            if (diff > M_PI) diff -= 2 * M_PI;
            if (diff < -M_PI) diff += 2 * M_PI;
            q_interp[j] = node1->q[j] + t * diff;
        }

        // Debugging output
        auto pos = RobotKinematics::computeFK(q_interp);
        std::cout << "Interpolated joint angles: ";
        for (double angle : q_interp) std::cout << angle << " ";
        std::cout << "\nInterpolated end-effector position: " << pos[0] << ", " << pos[1] << ", " << pos[2] << std::endl;

        // Check joint limits
        if (!RobotKinematics::isJointLimitValid(q_interp)) {
            return false;
        }
        // Check end-effector position against obstacles
        std::array<double, 3> point = {pos[0], pos[1], pos[2]};
        for (const auto& obstacle : obstacles) {
            bool inside = true;
            for (int j = 0; j < 3; ++j) {
                if (point[j] < obstacle.min_point[j] - safety_margin || 
                    point[j] > obstacle.max_point[j] + safety_margin) {
                    inside = false;
                    break;
                }
            }
            if (inside) return false;
        }
    }
    return true;
}

void RRTStar::rewire(const std::vector<Node*>& neighbors, Node* new_node) {
    for(auto* neighbor : neighbors) {
        if (isCollisionFree(neighbor, new_node)) {
            double tentative_cost = new_node->cost + distance(new_node, neighbor);
            if (tentative_cost < neighbor->cost) {
                neighbor->parent = new_node;
                neighbor->cost = tentative_cost;
            }
        }
    }
}

double RRTStar::distance(Node* node1, Node* node2) {
    double joint_dist = 0.0;
    for(size_t i = 0; i < 6; i++) {
        double diff = node2->q[i] - node1->q[i];
        // Handle angular wrap-around
        if(diff > M_PI) diff -= 2*M_PI;
        if(diff < -M_PI) diff += 2*M_PI;
        joint_dist += diff * diff;
    }
    
    // Combine joint space and task space distances
    auto pos1 = node1->end_effector_position();
    auto pos2 = node2->end_effector_position();
    double task_dist = std::pow(pos2[0] - pos1[0], 2) + 
                      std::pow(pos2[1] - pos1[1], 2) + 
                      std::pow(pos2[2] - pos1[2], 2);
    
    return std::sqrt(joint_dist) + 0.1 * std::sqrt(task_dist);
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
    
    // Use the correct result type that nanoflann expects
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

void RRTStar::getFinalPath(Node* goal_node, std::vector<Node*>& path) {
    Node* current = goal_node;
    while (current != nullptr) {
        path.push_back(current);
        current = current->parent;
    }
    std::reverse(path.begin(), path.end());
}

void RRTStar::optimizePath(std::vector<Node*>& path) {
    if (path.size() < 3) return;
    bool improved;
    do {
        improved = false;
        // Forward pass
        size_t i = 0;
        while (i < path.size() - 2) {
            if (isCollisionFree(path[i], path[i + 2])) {
                path.erase(path.begin() + i + 1);
                improved = true;
            } else {
                i++;
            }
        }
        // Backward pass to catch missed opportunities
        i = path.size() - 1;
        while (i >= 2) {
            if (isCollisionFree(path[i - 2], path[i])) {
                path.erase(path.begin() + i - 1);
                improved = true;
            }
            i--;
        }
    } while (improved); // Repeat until no more improvements
}

bool RRTStar::isStateValid(const std::array<double, 6>& q) {
    if (!RobotKinematics::isJointLimitValid(q)) {
        std::cout << "Joint limits violated!" << std::endl;
        return false;
    }
    auto pos = RobotKinematics::computeFK(q);
    bool valid = pos[0] >= map_min_x && pos[0] <= (map_min_x + map_width) &&
                 pos[1] >= map_min_y && pos[1] <= (map_min_y + map_height) &&
                 pos[2] >= map_min_z && pos[2] <= (map_min_z + map_depth);
    if (!valid) {
        std::cout << "Position (" << pos[0] << ", " << pos[1] << ", " << pos[2] 
                  << ") is outside workspace bounds [" << map_min_x << "," << (map_min_x + map_width) 
                  << "] x [" << map_min_y << "," << (map_min_y + map_height)
                  << "] x [" << map_min_z << "," << (map_min_z + map_depth) << "]" << std::endl;
    }
    return valid;
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
    // Set backend explicitly for headless environments
    plt::backend("Agg");

    // Create a new figure
    plt::figure_size(1200, 800);

    // Plot tree nodes and edges
    std::vector<double> x_coords, y_coords, z_coords;
    for (const auto& node : nodes) {
        x_coords.push_back(node->x);
        y_coords.push_back(node->y);
        z_coords.push_back(node->z);
    }

    // Plot tree edges
    for (const auto& node : nodes) {
        if (node->parent != nullptr) {
            std::vector<double> edge_x = {node->x, node->parent->x};
            std::vector<double> edge_y = {node->y, node->parent->y};
            std::vector<double> edge_z = {node->z, node->parent->z};
            plt::plot3(edge_x, edge_y, edge_z, {{"color", "lightblue"}, {"linestyle", "-"}, {"linewidth", "0.5"}});
        }
    }

    // Plot all nodes
    plt::plot3(x_coords, y_coords, z_coords, {{"color", "blue"}, {"marker", "."}, 
        {"linestyle", "none"}, {"markersize", "5"}, {"label", "Tree Nodes"}});

    // Plot original path
    if (!path.empty()) {
        std::vector<double> path_x, path_y, path_z;
        for (const auto& node : path) {
            path_x.push_back(node->x);
            path_y.push_back(node->y);
            path_z.push_back(node->z);
        }
        plt::plot3(path_x, path_y, path_z, {{"color", "orange"}, {"linestyle", "--"}, 
            {"linewidth", "2.0"}, {"label", "Original Path"}});
    }

    // Plot start and goal
    std::vector<double> start_xyz = {start_node->x, start_node->y, start_node->z};
    plt::plot3(std::vector<double>{start_xyz[0]}, 
               std::vector<double>{start_xyz[1]}, 
               std::vector<double>{start_xyz[2]}, 
               {{"color", "green"}, {"marker", "*"}, {"markersize", "15"}, {"label", "Start"}});

    std::vector<double> goal_xyz = {goal_node->x, goal_node->y, goal_node->z};
    plt::plot3(std::vector<double>{goal_xyz[0]}, 
               std::vector<double>{goal_xyz[1]}, 
               std::vector<double>{goal_xyz[2]}, 
               {{"color", "red"}, {"marker", "*"}, {"markersize", "15"}, {"label", "Goal"}});

    // Plot obstacles
    for (const auto& obstacle : obstacles) {
        // Plot each face of the box
        std::array<double, 8> x_corners = {
            obstacle.min_point[0], obstacle.max_point[0],
            obstacle.max_point[0], obstacle.min_point[0],
            obstacle.min_point[0], obstacle.max_point[0],
            obstacle.max_point[0], obstacle.min_point[0]
        };
        std::array<double, 8> y_corners = {
            obstacle.min_point[1], obstacle.min_point[1],
            obstacle.max_point[1], obstacle.max_point[1],
            obstacle.min_point[1], obstacle.min_point[1],
            obstacle.max_point[1], obstacle.max_point[1]
        };
        std::array<double, 8> z_corners = {
            obstacle.min_point[2], obstacle.min_point[2],
            obstacle.min_point[2], obstacle.min_point[2],
            obstacle.max_point[2], obstacle.max_point[2],
            obstacle.max_point[2], obstacle.max_point[2]
        };
        // Plot box edges
        for (int i = 0; i < 4; ++i) {
            int j = (i + 1) % 4;
            std::vector<double> edge_x = {x_corners[i], x_corners[j]};
            std::vector<double> edge_y = {y_corners[i], y_corners[j]};
            std::vector<double> edge_z = {z_corners[i], z_corners[j]};
            plt::plot3(edge_x, edge_y, edge_z, {{"color", "black"}, {"linestyle", "-"}, {"alpha", "0.5"}});
            // Vertical edges
            std::vector<double> v_edge_x = {x_corners[i], x_corners[i+4]};
            std::vector<double> v_edge_y = {y_corners[i], y_corners[i+4]};
            std::vector<double> v_edge_z = {z_corners[i], z_corners[i+4]};
            plt::plot3(v_edge_x, v_edge_y, v_edge_z, {{"color", "black"}, {"linestyle", "-"}, {"alpha", "0.5"}});
        }
    }

    // Add labels, title, grid, and legend
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::set_zlabel("Z");
    plt::title("RRT* Path Planning in 3D");
    plt::grid(true);
    plt::legend();

    // Save the plot to a file
    plt::save("/home/thornch/Documents/Cpp/PathPlanning/rrtstar_sixDoF1/path_plot.png");
    std::cout << "Plot saved to path_plot.png" << std::endl;

    // Close the figure to free resources
    plt::close();
}

// void RRTStar::plotCuboid(const std::array<double, 3>& min_point,
//                         const std::array<double, 3>& max_point,
//                         const std::map<std::string, std::string>& style) {
//     // Plot each face of the cuboid
//     double x_min = min_point[0], y_min = min_point[1], z_min = min_point[2];
//     double x_max = max_point[0], y_max = max_point[1], z_max = max_point[2];

//     // Bottom face
//     std::vector<double> x = {x_min, x_max, x_max, x_min, x_min};
//     std::vector<double> y = {y_min, y_min, y_max, y_max, y_min};
//     std::vector<double> z(5, z_min);
//     plt::plot3(x, y, z, style);

//     // Top face
//     std::vector<double> z_top(5, z_max);
//     plt::plot3(x, y, z_top, style);

//     // Vertical edges
//     for (int i = 0; i < 4; ++i) {
//         std::vector<double> x_edge = {x[i]};
//         std::vector<double> y_edge = {y[i]};
//         std::vector<double> z_edge = {z_min, z_max};
//         plt::plot3(x_edge, y_edge, z_edge, style);
//     }
// }

// void exportData(const std::vector<Node*>& path, const std::vector<Obstacle>& obstacles, const std::string& filename) {
//     std::ofstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open file for exporting data." << std::endl;
//         return;
//     }
    
//     // Write original path
//     file << "original_path\n";
//     for (const auto& node : path) {
//         file << node->x << "," << node->y << "," << node->z << "\n";
//     }
    
//     // Write smoothed path (if applicable)
//     file << "smoothed_path\n";
//     for (const auto& node : path) {
//         file << node->x << "," << node->y << "," << node->z << "\n";
//     }
    
//     // Write obstacles
//     file << "obstacles\n";
//     for (const auto& obstacle : obstacles) {
//         file << obstacle.min_point[0] << "," << obstacle.min_point[1] << "," << obstacle.min_point[2] << ","
//              << obstacle.max_point[0] << "," << obstacle.max_point[1] << "," << obstacle.max_point[2] << "\n";
//     }
//     file.close();
// }

// void exportData(const std::vector<Node*>& original_path, 
//                 const std::vector<Node*>& smoothed_path,
//                 const std::vector<Obstacle>& obstacles, 
//                 const std::string& filename) {
//     std::ofstream file(filename);
//     if (!file.is_open()) {
//         std::cerr << "Failed to open file for exporting data: " << filename << std::endl;
//         return;
//     }
    
//     // Write original path
//     file << "original_path\n";
//     for (const auto* node : original_path) {
//         file << node->x << "," << node->y << "," << node->z << "\n";
//     }
    
//     // Write smoothed path
//     file << "\nsmoothed_path\n";
//     for (const auto* node : smoothed_path) {
//         file << node->x << "," << node->y << "," << node->z << "\n";
//     }
    
//     // Write obstacles
//     file << "\nobstacles\n";
//     for (const auto& obstacle : obstacles) {
//         file << obstacle.min_point[0] << "," << obstacle.min_point[1] << "," << obstacle.min_point[2] << ","
//              << obstacle.max_point[0] << "," << obstacle.max_point[1] << "," << obstacle.max_point[2] << "\n";
//     }
    
//     file.flush();  // Ensure all data is written
//     file.close();  // Explicitly close the file

//     if (file.fail()) {
//         std::cerr << "Error occurred while writing to file" << std::endl;
//     }
// }

void RRTStar::savePlanningResults(const std::vector<Node*>& path) {
    // Save path data
    exportPathData(path, "/home/thornch/Documents/Cpp/PathPlanning/rrtstar_sixDoF1/path_data.csv");

    // Print summary
    std::cout << "\nPath Summary:" << std::endl;
    std::cout << "Number of nodes: " << path.size() << std::endl;
    std::cout << "Start position: (" 
              << path.front()->x << ", " 
              << path.front()->y << ", " 
              << path.front()->z << ")" << std::endl;
    std::cout << "End position: (" 
              << path.back()->x << ", " 
              << path.back()->y << ", " 
              << path.back()->z << ")" << std::endl;
    
    // Calculate total path length
    double total_distance = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        double dx = path[i]->x - path[i-1]->x;
        double dy = path[i]->y - path[i-1]->y;
        double dz = path[i]->z - path[i-1]->z;
        total_distance += std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    std::cout << "Total path length: " << total_distance << " units" << std::endl;
}

void RRTStar::exportPathData(const std::vector<Node*>& path, const std::string& filename) {
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filename << std::endl;
        return;
    }

    // Write header
    file << "node_id,x,y,z,joint1,joint2,joint3,joint4,joint5,joint6" << std::endl;

    // Write path data
    for (size_t i = 0; i < path.size(); ++i) {
        const auto* node = path[i];
        file << i << ","
             << node->x << "," 
             << node->y << "," 
             << node->z << ","
             << node->q[0] << ","
             << node->q[1] << ","
             << node->q[2] << ","
             << node->q[3] << ","
             << node->q[4] << ","
             << node->q[5] << std::endl;
    }

    file.close();
    std::cout << "Path data exported to: " << filename << std::endl;
}



// void RRTStar::smoothPath(std::vector<Node*>& path) {
//     if (path.size() < 3) return;

//     // For 6DOF, we'll do simple linear interpolation
//     std::vector<Node*> smoothed_path;
//     smoothed_path.push_back(path.front());  // Keep start node

//     for (size_t i = 1; i < path.size(); ++i) {
//         Node* prev = path[i-1];
//         Node* curr = path[i];
        
//         // Create intermediate nodes
//         const int steps = 5;
//         for (int j = 1; j < steps; ++j) {
//             double t = static_cast<double>(j) / steps;
//             std::array<double, 6> interp_q;
            
//             // Interpolate joint values
//             for (size_t k = 0; k < 6; ++k) {
//                 double diff = curr->q[k] - prev->q[k];
//                 // Handle angular wrap-around
//                 if (diff > M_PI) diff -= 2*M_PI;
//                 if (diff < -M_PI) diff += 2*M_PI;
//                 interp_q[k] = prev->q[k] + t * diff;
//             }
            
//             // Create new node if collision-free
//             auto new_node = std::make_unique<Node>(interp_q);
//             if (isCollisionFree(smoothed_path.back(), new_node.get())) {
//                 smoothed_path.push_back(new_node.get());
//                 node_storage.push_back(std::move(new_node));
//             }
//         }
        
//         smoothed_path.push_back(curr);
//     }

//     path = smoothed_path;
// }

// Modify the DH parameters to produce a more constrained workspace
// std::array<DHParameters, 6> RobotKinematics::dh_params = {
//     DHParameters(0,      M_PI/2,  50,   0),  // Base to shoulder - increased Z offset
//     DHParameters(40,     0,       0,    0),  // Shoulder to elbow
//     DHParameters(40,     0,       0,    0),  // Elbow to wrist
//     DHParameters(0,      M_PI/2,  20,   0),  // Wrist 1
//     DHParameters(0,     -M_PI/2,  0,    0),  // Wrist 2
//     DHParameters(0,      0,       15,   0)   // End effector
// };