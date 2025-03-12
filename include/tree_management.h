#ifndef TREE_MANAGEMENT_H
#define TREE_MANAGEMENT_H

#include <vector>
#include <memory>
#include <array>
#include <random>
#include "nanoflann.hpp"
#include "robot_kinematics.h"

// Node structure for the RRT tree
struct Node : public std::enable_shared_from_this<Node> {
    std::array<double, 6> q;           // Joint configuration
    std::weak_ptr<Node> parent;        // Parent node
    double cost = 0.0;                 // Cost from start

    Node(const std::array<double, 6>& q_in) : q(q_in) {}

    // Get shared pointer to this node
    std::shared_ptr<Node> ptr() { 
        return shared_from_this();
    }
};

class TreeManager {
private:
    // Tree nodes
    std::vector<std::shared_ptr<Node>> nodes;
    std::shared_ptr<Node> start_node;
    std::shared_ptr<Node> goal_node;
    std::array<double, 6> goal_config;
    
    // Random number generation
    std::mt19937 gen;
    std::uniform_real_distribution<> dis_x, dis_y, dis_z;
    
    // Space parameters
    double map_width, map_height, map_depth;
    double map_min_x, map_min_y, map_min_z;
    
    // Planning parameters
    double step_size;
    double neighbor_radius;
    double goal_bias;
    
    // KD-tree for efficient nearest-neighbor search
    struct NodeAdapter {
        std::vector<std::shared_ptr<Node>>& nodes;
        NodeAdapter(std::vector<std::shared_ptr<Node>>& nodes) : nodes(nodes) {}
        
        inline size_t kdtree_get_point_count() const { 
            return nodes.size(); 
        }
        
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return nodes[idx]->q[dim];
        }
        
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { 
            return false; 
        }
    };
    
    NodeAdapter node_adapter;
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, NodeAdapter>, 
        NodeAdapter, 
        6, 
        size_t>;
    std::unique_ptr<KDTree> kdtree;
    
    int nodes_since_rebuild;
    static constexpr int REBUILD_THRESHOLD = 1500;
    
    // Adaptive goal bias parameters
    static constexpr double initial_goal_bias = 0.1;
    static constexpr double max_goal_bias = 0.5;

public:
    TreeManager(
        const std::array<double, 6>& start_q,
        const std::array<double, 6>& goal_q,
        double map_width, double map_height, double map_depth,
        double step_size, double neighbor_radius,
        double min_x = -500, double min_y = -500, double min_z = 0
    );
    
    ~TreeManager() = default;
    
    // Basic tree operations
    void rebuildKDTree();
    void addNode(std::shared_ptr<Node> node);
    std::shared_ptr<Node> getStartNode() const;
    std::shared_ptr<Node> getGoalNode() const;
    const std::array<double, 6>& getGoalConfig() const;
    size_t getNodeCount() const;
    
    // Node search operations
    std::shared_ptr<Node> nearest(std::shared_ptr<Node> target);
    std::vector<std::shared_ptr<Node>> radiusSearch(std::shared_ptr<Node> target, double radius);
    
    // Random node generation
    std::shared_ptr<Node> getRandomNode(int iteration_count, int max_iterations);
    
    // Distance calculation
    static double distance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2);
    
    // Path extraction
    void getFinalPath(std::shared_ptr<Node> goal_node, std::vector<std::shared_ptr<Node>>& path);
    std::vector<std::shared_ptr<Node>> createReturnPathSimple(const std::vector<std::shared_ptr<Node>>& forward_path);
    
    // Access to all nodes
    const std::vector<std::shared_ptr<Node>>& getAllNodes() const;
};

#endif // TREE_MANAGEMENT_H