// #ifndef RRTSTAR_MAIN_H
// #define RRTSTAR_MAIN_H

// #include <vector>
// #include <memory>
// #include <array>
// #include <Eigen/Geometry>
// #include "tree_management.h"
// #include "collision_detection.h"
// #include "path_optimization.h"
// #include "path_planning.h"
// // Visualization excluded for now

// // Define workspace limits that match those in InverseKinematics
// // This avoids direct access to private members
// namespace RobotWorkspace {
//     // Workspace boundaries (manually set to match InverseKinematics)
//     static constexpr double MIN_X = -1500.0; // mm
//     static constexpr double MAX_X = 1500.0;  // mm
//     static constexpr double MIN_Y = -1500.0; // mm
//     static constexpr double MAX_Y = 1500.0;  // mm
//     static constexpr double MIN_Z = -500.0;  // mm
//     static constexpr double MAX_Z = 2000.0;  // mm
    
//     // Derived dimensions
//     static constexpr double WIDTH = MAX_X - MIN_X;
//     static constexpr double HEIGHT = MAX_Y - MIN_Y;
//     static constexpr double DEPTH = MAX_Z - MIN_Z;
// }

// class RRTStarModified {
// private:
//     // Core components
//     std::unique_ptr<TreeManager> tree_manager;
//     std::unique_ptr<CollisionDetection> collision_detector;
//     std::unique_ptr<PathOptimization> path_optimizer;
//     std::unique_ptr<PathPlanner> path_planner;
//     // Visualization excluded for now
//     // std::unique_ptr<Visualization> visualizer;
    
//     // Planning parameters
//     double map_width, map_height, map_depth;
//     double map_min_x, map_min_y, map_min_z;
//     double step_size, neighbor_radius, safety_margin;
//     int max_iter;
    
//     // Start and goal configurations
//     std::array<double, 6> start_config;
//     std::array<double, 6> goal_config;

// public:
//     // Constructor with all parameters, using workspace limits from RobotWorkspace namespace
//     // Fixed parameter types to match what's being passed in tests
//     RRTStarModified(
//         const std::array<double, 6>& start_q, 
//         const std::array<double, 6>& goal_q,
//         double map_width, double map_height, double map_depth,
//         double step_size, double neighbor_radius,
//         double safety_margin, int max_iter,
//         // Use the workspace limits from RobotWorkspace namespace by default
//         double min_x = RobotWorkspace::MIN_X, 
//         double min_y = RobotWorkspace::MIN_Y, 
//         double min_z = RobotWorkspace::MIN_Z
//     );
    
//     ~RRTStarModified();
    
//     // Main interface functions
//     std::vector<std::shared_ptr<Node>> findPath();
//     void visualizePath(const std::vector<std::shared_ptr<Node>>& path);
//     void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path, 
//                           std::vector<std::array<double, 8>>& robot_commands);
//     std::vector<std::shared_ptr<Node>> createReturnPathSimple(
//         const std::vector<std::shared_ptr<Node>>& forward_path);
    
//     // Configuration functions
//     void setVisualizationEnabled(bool enabled);
//     void setGoalPose(const Eigen::Isometry3d& pose);
//     const Eigen::Isometry3d& getGoalPose() const;
    
//     // Obstacle management - updated to use Obstacle class
//     static void updateObstacles(const std::vector<Obstacle>& new_obstacles);
//     static const std::vector<Obstacle>& getObstacles();
    
//     // Quality evaluation
//     PathQualityMetrics evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path);
// };

// #endif // RRTSTAR_MAIN_H


#ifndef RRTSTAR_MAIN_H
#define RRTSTAR_MAIN_H

#include <vector>
#include <memory>
#include <array>
#include <Eigen/Geometry>
#include "tree_management.h"
#include "collision_detection.h"
#include "path_optimization.h"
#include "path_planning.h"
#include "workspace_limits.h" // Use the shared workspace limits
#include "path_export.h"
#include "path_return.h"
// Visualization excluded for now

class RRTStarModified {
private:
    // Core components
    std::unique_ptr<TreeManager> tree_manager;
    std::unique_ptr<CollisionDetection> collision_detector;
    std::unique_ptr<PathOptimization> path_optimizer;
    std::unique_ptr<PathPlanner> path_planner;
    // Visualization excluded for now
    // std::unique_ptr<Visualization> visualizer;
    
    // Planning parameters
    double map_width, map_height, map_depth;
    double map_min_x, map_min_y, map_min_z;
    double step_size, neighbor_radius, safety_margin;
    int max_iter;
    
    // Start and goal configurations
    std::array<double, 6> start_config;
    std::array<double, 6> goal_config;

public:
    // Constructor with all parameters, using workspace limits from shared header
    // Fixed parameter types to match what's being passed in tests
    RRTStarModified(
        const std::array<double, 6>& start_q, 
        const std::array<double, 6>& goal_q,
        double map_width, double map_height, double map_depth,
        double step_size, double neighbor_radius,
        double safety_margin, int max_iter,
        // Use the workspace limits from the shared header by default
        double min_x = WorkspaceLimits::MIN_X, 
        double min_y = WorkspaceLimits::MIN_Y, 
        double min_z = WorkspaceLimits::MIN_Z
    );
    
    ~RRTStarModified();
    
    // Main interface functions
    std::vector<std::shared_ptr<Node>> findPath();
    void visualizePath(const std::vector<std::shared_ptr<Node>>& path);
    
    
    // Configuration functions
    void setVisualizationEnabled(bool enabled);
    void setGoalPose(const Eigen::Isometry3d& pose);
    const Eigen::Isometry3d& getGoalPose() const;
    
    // Obstacle management - updated to use Obstacle class
    static void updateObstacles(const std::vector<Obstacle>& new_obstacles);
    static const std::vector<Obstacle>& getObstacles();

    //---------Path export && return--------
    void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path, 
                            std::vector<std::array<double, 8>>& robot_commands);
    std::vector<std::shared_ptr<Node>> createReturnPathSimple(const std::vector<std::shared_ptr<Node>>& forward_path);
    
    // Quality evaluation
    // PathQualityMetrics evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path);
};

#endif // RRTSTAR_MAIN_H