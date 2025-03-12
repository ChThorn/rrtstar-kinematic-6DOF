#ifndef PATH_PLANNING_H
#define PATH_PLANNING_H

#include <vector>
#include <memory>
#include <Eigen/Geometry>
#include "tree_management.h"
#include "collision_detection.h"
#include "path_optimization.h"

class PathPlanner {
private:
    TreeManager& tree_manager;
    CollisionDetection& collision_detector;
    PathOptimization& path_optimizer;
    
    // Planning parameters
    double step_size;
    double neighbor_radius;
    int max_iter;
    
    // Goal parameters
    Eigen::Isometry3d goal_pose;
    
    // Helper methods
    std::shared_ptr<Node> steer(std::shared_ptr<Node> from_node, std::shared_ptr<Node> to_node);
    void rewire(const std::vector<std::shared_ptr<Node>>& neighbors, std::shared_ptr<Node> new_node);

public:
    PathPlanner(
        TreeManager& tree_manager,
        CollisionDetection& collision_detector,
        PathOptimization& path_optimizer,
        double step_size,
        double neighbor_radius,
        int max_iter
    );
    ~PathPlanner() = default;
    
    // Main planning functions
    std::vector<std::shared_ptr<Node>> globalPlanner();
    void localPlanner(std::vector<std::shared_ptr<Node>>& path);
    
    // Full planning pipeline
    std::vector<std::shared_ptr<Node>> findPath();
    
    // Goal pose management
    void setGoalPose(const Eigen::Isometry3d& pose);
    const Eigen::Isometry3d& getGoalPose() const;
};

#endif // PATH_PLANNING_H