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
    // Constructor with all parameters
    RRTStarModified(
        const std::array<double, 6>& start_q, 
        const std::array<double, 6>& goal_q,
        double map_width, double map_height, double map_depth,
        double step_size, double neighbor_radius,
        double safety_margin, int max_iter,
        double min_x = -500, double min_y = -500, double min_z = 0
    );
    
    ~RRTStarModified();
    
    // Main interface functions
    std::vector<std::shared_ptr<Node>> findPath();
    void visualizePath(const std::vector<std::shared_ptr<Node>>& path);
    void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path, 
                          std::vector<std::array<double, 8>>& robot_commands);
    std::vector<std::shared_ptr<Node>> createReturnPathSimple(
        const std::vector<std::shared_ptr<Node>>& forward_path);
    
    // Configuration functions
    void setVisualizationEnabled(bool enabled);
    void setGoalPose(const Eigen::Isometry3d& pose);
    const Eigen::Isometry3d& getGoalPose() const;
    
    // Obstacle management
    static void updateObstacles(const std::vector<PlanningObstacle>& new_obstacles);
    static const std::vector<PlanningObstacle>& getObstacles();
    
    // Quality evaluation
    PathQualityMetrics evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path);
};

#endif // RRTSTAR_MAIN_H