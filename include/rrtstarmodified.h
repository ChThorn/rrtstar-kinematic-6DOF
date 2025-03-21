#ifndef RRTStarModified_H
#define RRTStarModified_H

#include <array>
#include <vector>
#include <memory>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>
#include "nanoflann.hpp"
#include "matplotlibcpp.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

//
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "ik_solution_evaluator.h"

namespace plt = matplotlibcpp;

// Robot parameters (declared only once)
// constexpr double LINK_LENGTHS[] = {0.0892, 0.425, 0.392, 0.1093, 0.09475, 0.0825};

// Robot link parameters from your kinematics implementation
constexpr double LINK_LENGTHS[] = {
    ForwardKinematics::d1,  // 169.2 mm
    ForwardKinematics::a1,  // 425.0 mm
    ForwardKinematics::a2,  // 392.0 mm
    ForwardKinematics::d4,  // 110.7 mm
    ForwardKinematics::d5,  // 110.7 mm
    ForwardKinematics::d6   // 96.7 mm
};

namespace RobotKinematics {
    // Forward kinematics - returns a transformation matrix
    Eigen::Isometry3d computeFK(const std::array<double, 6>& q);
    
    // Numerical IK - declaration matches implementation
    std::array<double, 6> inverseKinematics(const Eigen::Vector3d& target_pos,
                                          const std::array<double, 6>& q_init,
                                          double tol = 1e-3,
                                          int max_iter = 100);

    // New function for IK with orientation
    std::array<double, 6> inverseKinematicsWithOrientation(
        const Eigen::Isometry3d& target_pose,
        const std::array<double, 6>& q_init,
        double tol = 1e-3,
        int max_iter = 100);
}

struct Node : public std::enable_shared_from_this<Node> { // Add inheritance
    std::array<double, 6> q;
    std::weak_ptr<Node> parent;
    double cost = 0.0;

    Node(const std::array<double, 6>& q_in) : q(q_in) {}

    // Simplify ptr() implementation
    std::shared_ptr<Node> ptr() { 
        return shared_from_this();
    }
};

struct PathQualityMetrics {
    double total_length;
    double max_step;
    double avg_step;
    double smoothness;

    PathQualityMetrics() : total_length(0.0), max_step(0.0), avg_step(0.0), smoothness(0.0){}
};

struct PlanningObstacle {
    std::array<double, 3> min_point;
    std::array<double, 3> max_point;
    PlanningObstacle(const std::array<double, 3>& min, const std::array<double, 3>& max)
        : min_point(min), max_point(max) {}
};

class RRTStarModified {
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
    std::vector<std::shared_ptr<Node>> nodes;

    // KD-tree for efficient nearest-neighbor search
    struct NodeAdapter {
        std::vector<std::shared_ptr<Node>>& nodes;
        NodeAdapter(std::vector<std::shared_ptr<Node>>& nodes) : nodes(nodes) {}
        inline size_t kdtree_get_point_count() const { return nodes.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return nodes[idx]->q[dim];
        }
        template <class BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    NodeAdapter node_adapter;
    using KDTree = nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, NodeAdapter>, 
        NodeAdapter, 
        6, 
        size_t>;
    std::unique_ptr<KDTree> kdtree;

    std::shared_ptr<Node> start_node;
    std::shared_ptr<Node> goal_node;

    // Static obstacle list
    static std::vector<PlanningObstacle> obstacles;

    // Helper functions
    std::shared_ptr<Node> steer(std::shared_ptr<Node> nearest_node, std::shared_ptr<Node> random_node);
    std::pair<bool, int> isCollisionFree(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2);
    void rewire(const std::vector<std::shared_ptr<Node>>& neighbors, std::shared_ptr<Node> new_node);
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

    // bool visualization_enabled = true;

    int nodes_since_rebuild = 0;
    static constexpr int REBUILD_THRESHOLD = 1500; // Adjust based on testing
    void rebuildKDTree();

    // Helper functions
    static double wrapAngle(double angle)
    {
        return angle - 2*M_PI * std::floor((angle + M_PI) / (2*M_PI));
    }
    static double clampToJointLimits(int joint_idx, double value)
    {
        return std::clamp(value,
                          joint_limits_min[joint_idx],
                          joint_limits_max[joint_idx]);
    }

    #ifdef USE_MATPLOTLIB
    bool visualization_enabled = true;
    #else
    bool visualization_enabled = false;
    #endif

    Eigen::Isometry3d goal_pose; 

public:
    // Robot parameters for kinematics
    static constexpr double MAX_JOINT_VEL = M_PI/4;  // rad/s (45 deg/s)
    static constexpr double CONTROL_RATE = 100.0;    // Hz

    PathQualityMetrics evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path);
    RRTStarModified(const std::array<double, 6>& start_q, 
                    const std::array<double, 6>& goal_q,
                    double map_width, double map_height, double map_depth,
                    double step_size, double neighbor_radius,
                    double safety_margin, int max_iter,
                    double min_x = -500, double min_y = -500, double min_z = 0);
    ~RRTStarModified();

    std::vector<std::shared_ptr<Node>> findPath();
    void getFinalPath(std::shared_ptr<Node> goal_node, std::vector<std::shared_ptr<Node>>& path);
    void optimizePath(std::vector<std::shared_ptr<Node>>& path);
    bool isStateValid(const std::array<double, 6>& q);
    std::shared_ptr<Node> nearest(std::shared_ptr<Node> target);
    std::vector<std::shared_ptr<Node>> radiusSearch(std::shared_ptr<Node> target, double radius);
    double distance(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2);

    void visualizePath(const std::vector<std::shared_ptr<Node>>& path);

    // Getter for obstacles
    static const std::vector<PlanningObstacle>& getObstacles() {
        return obstacles;
    }

    bool testLineAABBIntersection(const std::array<double, 3>& start,
                                  const std::array<double, 3>& end,
                                  const std::array<double, 3>& box_min,
                                  const std::array<double, 3>& box_max) {
        return lineAABBIntersection(start, end, box_min, box_max);
    }

    // Joint limits
    // static constexpr std::array<double, 6> joint_limits_min = {-M_PI, -M_PI, -M_PI, -M_PI, -M_PI, -M_PI};
    // static constexpr std::array<double, 6> joint_limits_max = {M_PI, M_PI, M_PI, M_PI, M_PI, M_PI};

    // Joint limits from your InverseKinematics implementation
    static constexpr std::array<double, 6> joint_limits_min = {
        InverseKinematics::joint_limits_min[0],  // Base
        InverseKinematics::joint_limits_min[1],  // Shoulder
        InverseKinematics::joint_limits_min[2],  // Elbow
        InverseKinematics::joint_limits_min[3],  // Wrist 1
        InverseKinematics::joint_limits_min[4],  // Wrist 2
        InverseKinematics::joint_limits_min[5]   // Wrist 3
    };

    static constexpr std::array<double, 6> joint_limits_max = {
        InverseKinematics::joint_limits_max[0],  // Base
        InverseKinematics::joint_limits_max[1],  // Shoulder
        InverseKinematics::joint_limits_max[2],  // Elbow
        InverseKinematics::joint_limits_max[3],  // Wrist 1
        InverseKinematics::joint_limits_max[4],  // Wrist 2
        InverseKinematics::joint_limits_max[5]   // Wrist 3
    };

    const std::array<double, 6>& getGoalConfig() const {
        return goal_config;
    }

    int testIsCollisionFree(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) {
        auto [collision_free, steps] = isCollisionFree(node1, node2);
        return steps; // Return the number of interpolation steps
    }

    std::shared_ptr<Node> getRandomNode(int i);

    void setVisualizationEnabled(bool enabled) {
        visualization_enabled = enabled;
    }

    std::vector<std::shared_ptr<Node>> globalPlanner();
    void localPlanner(std::vector<std::shared_ptr<Node>>& path);
    void refinePathDynamically(std::vector<std::shared_ptr<Node>>& path);
    std::array<double, 6> inverseKinematics(const std::array<double, 3>& pos);
    void applyCubicSpline(const std::vector<std::array<double, 3>>& points);
    void applyQuinticSpline(std::vector<std::shared_ptr<Node>>& path);
    void refinePathWithIK(std::vector<std::shared_ptr<Node>>& path);
    void applyQuinticSplineWithConstraints(std::vector<std::shared_ptr<Node>>& path);

    static void updateObstacles(const std::vector<PlanningObstacle>& new_obstacles) {
        obstacles = new_obstacles;
    }

    std::array<double, 6> inverseKinematicsWithPose(
        const Eigen::Isometry3d& target_pose, 
        const std::array<double, 6>& initial_guess);

    // Initialize goal pose from the goal configuration
    void setGoalPose(const Eigen::Isometry3d& pose) {
        goal_pose = pose;
    }

    const Eigen::Isometry3d& getGoalPose() const {
        return goal_pose;
    }

    // Path export method
    void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path,
                            std::vector<std::array<double, 8>>& robot_commands);

    // Reverse path from goal to home
    std::vector<std::shared_ptr<Node>> createReturnPathSimple(const std::vector<std::shared_ptr<Node>>& forward_path);

};

#endif // RRTStarModified_H