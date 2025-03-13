#include "tree_management.h"
#include <algorithm>
#include <cmath>
#include <random>
#include <stdexcept>

TreeManager::TreeManager(
    const std::array<double, 6>& start_q,
    const std::array<double, 6>& goal_q,
    double map_width, double map_height, double map_depth,
    double step_size, double neighbor_radius,
    double min_x, double min_y, double min_z)
    : map_width(map_width), map_height(map_height), map_depth(map_depth),
      map_min_x(min_x), map_min_y(min_y), map_min_z(min_z),
      step_size(step_size), neighbor_radius(neighbor_radius),
      goal_bias(initial_goal_bias),
      gen(std::random_device{}()),
      dis_x(min_x, min_x + map_width), 
      dis_y(min_y, min_y + map_height), 
      dis_z(min_z, min_z + map_depth),
      goal_config(goal_q),
      node_adapter(nodes),
      nodes_since_rebuild(0) {
    
    start_node = std::make_shared<Node>(start_q);
    goal_node = std::make_shared<Node>(goal_q);

    nodes.push_back(start_node);
    
    kdtree = std::make_unique<KDTree>(
        6, 
        node_adapter, 
        nanoflann::KDTreeSingleIndexAdaptorParams(10 /* max leaf size */)
    );
    rebuildKDTree();
}

void TreeManager::rebuildKDTree() {
    kdtree = std::make_unique<KDTree>(
        6, 
        node_adapter, 
        nanoflann::KDTreeSingleIndexAdaptorParams(10)
    );
    kdtree->buildIndex(); // Full build from all nodes
}

void TreeManager::addNode(std::shared_ptr<Node> node) {
    nodes.push_back(node);
    
    nodes_since_rebuild++;
    if (nodes_since_rebuild >= REBUILD_THRESHOLD) {
        rebuildKDTree();
        nodes_since_rebuild = 0;
    }
}

std::shared_ptr<Node> TreeManager::getStartNode() const {
    return start_node;
}

std::shared_ptr<Node> TreeManager::getGoalNode() const {
    return goal_node;
}

const std::array<double, 6>& TreeManager::getGoalConfig() const {
    return goal_config;
}

size_t TreeManager::getNodeCount() const {
    return nodes.size();
}

std::shared_ptr<Node> TreeManager::nearest(std::shared_ptr<Node> target) {
    double query_pt[6];
    for (size_t i = 0; i < 6; ++i) {
        query_pt[i] = target->q[i];
    }
    
    size_t index;
    double min_dist_sq;
    nanoflann::KNNResultSet<double> resultSet(1);
    resultSet.init(&index, &min_dist_sq);
    kdtree->findNeighbors(resultSet, query_pt, nanoflann::SearchParameters(10));
    
    return nodes[index];
}

std::vector<std::shared_ptr<Node>> TreeManager::radiusSearch(
    std::shared_ptr<Node> target, double radius) 
{
    double query_pt[6];
    for (size_t i = 0; i < 6; ++i) {
        query_pt[i] = target->q[i];
    }

    std::vector<nanoflann::ResultItem<size_t, double>> matches;
    nanoflann::SearchParameters params;
    params.sorted = true;

    kdtree->radiusSearch(query_pt, radius*radius, matches, params);

    std::vector<std::shared_ptr<Node>> result;
    for (const auto& match : matches) {
        result.push_back(nodes[match.first]);
    }
    return result;
}

std::shared_ptr<Node> TreeManager::getRandomNode(int iteration, int max_iterations) {
    // Adaptive goal bias based on iteration
    const double current_goal_bias = std::min(max_goal_bias, initial_goal_bias +
        (max_goal_bias - initial_goal_bias) * (iteration / static_cast<double>(max_iterations)));

    if (std::uniform_real_distribution<>(0.0, 1.0)(gen) < current_goal_bias) {
        return std::make_shared<Node>(goal_config);
    }

    std::array<double, 6> q;
    double heuristic_weight = 0.7; // Weight for heuristic bias
    
    for (size_t j = 0; j < 6; ++j) {
        double range = JOINT_LIMITS_MAX[j] - JOINT_LIMITS_MIN[j];
        double center = (JOINT_LIMITS_MAX[j] + JOINT_LIMITS_MIN[j]) / 2.0;
        double u = std::uniform_real_distribution<>(0.0, 1.0)(gen);
        double v = std::uniform_real_distribution<>(0.0, 1.0)(gen);

        // Bias sampling toward the goal using heuristic
        double heuristic_offset = heuristic_weight * (goal_config[j] - center);
        q[j] = center + (u < 0.5 ? 1 : -1) * range * (1.0 - std::sqrt(v)) / 2.0 + heuristic_offset;
        q[j] = std::clamp(q[j], JOINT_LIMITS_MIN[j], JOINT_LIMITS_MAX[j]);
    }
    
    return std::make_shared<Node>(q);
}

double TreeManager::distance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) {
    if(!node1 || !node2) return std::numeric_limits<double>::infinity();
    
    double dist = 0.0;
    for(size_t i = 0; i < 6; i++) {
        double diff = node2->q[i] - node1->q[i];
        if(i >= 3) {
            // Wrap angles for rotational joints
            diff = RobotKinematics::wrapAngle(diff);
        }
        dist += diff * diff;
    }
    return std::sqrt(dist);
}

void TreeManager::getFinalPath(std::shared_ptr<Node> goal_node, 
                             std::vector<std::shared_ptr<Node>>& path) {
    std::shared_ptr<Node> current = goal_node;
    
    while(current) {
        path.push_back(current);
        if(auto parent = current->parent.lock()) {
            current = parent;
        } else {
            current.reset();  // Break loop for orphaned nodes
        }
    }
    
    std::reverse(path.begin(), path.end());
}

// std::vector<std::shared_ptr<Node>> TreeManager::createReturnPathSimple(
//     const std::vector<std::shared_ptr<Node>>& forward_path) {
    
//     // Create a reversed copy of the forward path
//     std::vector<std::shared_ptr<Node>> return_path;
    
//     // Simply add nodes in reverse order
//     for (auto it = forward_path.rbegin(); it != forward_path.rend(); ++it) {
//         auto node = std::make_shared<Node>((*it)->q);
        
//         if (!return_path.empty()) {
//             node->parent = return_path.back();
//         }
        
//         return_path.push_back(node);
//     }
    
//     return return_path;
// }

const std::vector<std::shared_ptr<Node>>& TreeManager::getAllNodes() const {
    return nodes;
}