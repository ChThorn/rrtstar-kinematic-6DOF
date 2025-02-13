#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <array>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include "nanoflann.hpp"
#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

namespace RobotKinematics {
    inline std::array<double, 3> computeFK(const std::array<double, 6>& q) {
        // Example: Simple 2-link planar arm (for demonstration purposes)
        double l1 = 1.0; // Length of link 1
        double l2 = 1.0; // Length of link 2

        double x = l1 * std::cos(q[0]) + l2 * std::cos(q[0] + q[1]);
        double y = l1 * std::sin(q[0]) + l2 * std::sin(q[0] + q[1]);
        double z = 0.0; // Assuming 2D motion in the XY plane

        return {x, y, z};
    }
}


struct Node {
    std::array<double, 6> q;  // Joint angles // Later will considered with 6DOF (position and orientation)
    double x = 0, y = 0, z = 0;  // End-effector position (initialized to avoid warnings)
    Node* parent = nullptr;
    double cost = 0.0;

    Node(const std::array<double, 6>& q_in) : q(q_in) {}
};

struct PathQualityMetrics {
    double total_length;
    double max_step;
    double avg_step;
    double smoothness;

    PathQualityMetrics() : total_length(0.0), max_step(0.0), avg_step(0.0), smoothness(0.0){}
};

struct Obstacle {
    std::array<double, 3> min_point;
    std::array<double, 3> max_point;
    Obstacle(const std::array<double, 3>& min, const std::array<double, 3>& max)
        : min_point(min), max_point(max) {}
};

class RRTStar {
private:
    // Map parameters
    double map_width, map_height, map_depth;
    double map_min_x, map_min_y, map_min_z;
    double step_size, neighbor_radius, safety_margin;
    int max_iter;

    // Random number generation
    std::mt19937 gen;
    std::uniform_real_distribution<> dis_x, dis_y, dis_z;
    std::array<double, 6> goal_config;

    // Nodes and storage
    std::vector<Node*> nodes;
    std::vector<std::unique_ptr<Node>> node_storage;

    // KD-tree for efficient nearest-neighbor search
    struct NodeAdapter {
        std::vector<Node*>& nodes;
        NodeAdapter(std::vector<Node*>& nodes) : nodes(nodes) {}
        inline size_t kdtree_get_point_count() const { return nodes.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return nodes[idx]->q[dim];
        }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    NodeAdapter node_adapter;
    std::unique_ptr<nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, NodeAdapter>,
        NodeAdapter, 6>> kdtree;

    // Start and goal nodes
    std::unique_ptr<Node> start_node;
    std::unique_ptr<Node> goal_node;

    // Static obstacle list
    static std::vector<Obstacle> obstacles;

    // Helper functions
    
    std::unique_ptr<Node> steer(Node* nearest_node, Node* random_node);
    std::pair<bool, int> isCollisionFree(Node* node1, Node* node2);
    void rewire(const std::vector<Node*>& neighbors, Node* new_node);
    bool isObstacle(double x, double y, double z);
    bool lineAABBIntersection(const std::array<double, 3>& start,
                              const std::array<double, 3>& end,
                              const std::array<double, 3>& box_min,
                              const std::array<double, 3>& box_max);

    // Add adaptive goal bias parameters
    static constexpr double initial_goal_bias = 0.1; // Initial probability of sampling the goal
    static constexpr double max_goal_bias = 0.5;     // Maximum goal bias

    // Add dynamic interpolation parameters
    static constexpr int base_steps = 20;            // Base number of interpolation steps
    static constexpr double min_clearance = 10.0;    // Minimum safe distance from obstacles

    bool visualization_enabled = true;


public:
    PathQualityMetrics evaluatePathQuality(const std::vector<Node*>& path);
    RRTStar(const std::array<double, 6>& start_q, const std::array<double, 6>& goal_q,
            double map_width, double map_height, double map_depth,
            double step_size, double neighbor_radius,
            double safety_margin, int max_iter,
            double min_x = -500, double min_y = -500, double min_z = 0);
    ~RRTStar();

    std::vector<Node*> findPath();
    void getFinalPath(Node* goal_node, std::vector<Node*>& path);
    void optimizePath(std::vector<Node*>& path);
    bool isStateValid(const std::array<double, 6>& q);
    Node* nearest(Node* target);
    std::vector<Node*> radiusSearch(Node* target, double radius);
    double distance(Node* node1, Node* node2);

    void visualizePath(const std::vector<Node*>& path);

    // Getter for obstacles
    static const std::vector<Obstacle>& getObstacles() {
        return obstacles;
    }

    bool testLineAABBIntersection(const std::array<double, 3>& start,
                                  const std::array<double, 3>& end,
                                  const std::array<double, 3>& box_min,
                                  const std::array<double, 3>& box_max) {
        return lineAABBIntersection(start, end, box_min, box_max);
    }


    // Joint limits
    static constexpr std::array<double, 6> joint_limits_min = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    static constexpr std::array<double, 6> joint_limits_max = {10, 10, 10, M_PI, M_PI, M_PI};

    // static constexpr std::array<double, 6> joint_limits_min = {-170, -90, -170, -180, -120, -360};
    // static constexpr std::array<double, 6> joint_limits_max = {170, 130, 170, 180, 120, 360};

    const std::array<double, 6>& getGoalConfig() const {
        return goal_config;
    }

    int testIsCollisionFree(Node* node1, Node* node2) {
        auto [collision_free, steps] = isCollisionFree(node1, node2);
        return steps; // Return the number of interpolation steps
    }

    std::unique_ptr<Node> getRandomNode(int i);

    void setVisualizationEnabled(bool enabled) {
        visualization_enabled = enabled;
    }

    std::vector<Node*> globalPlanner();
    void localPlanner(std::vector<Node*>& path);
    void refinePathDynamically(std::vector<Node*>& path);
    std::array<double, 6> inverseKinematics(const std::array<double, 3>& pos);
    void applyCubicSpline(const std::vector<std::array<double, 3>>& points);
};

#endif // RRTSTAR_H