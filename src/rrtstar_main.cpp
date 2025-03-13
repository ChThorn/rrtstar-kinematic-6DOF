#include "rrtstar_main.h"
#include "robot_kinematics.h"
#include <stdexcept>
#include <iostream>
#include "path_export.h"      // Added for the extracted function
#include "path_return.h"      // Added for the extracted function

RRTStarModified::RRTStarModified(
    const std::array<double, 6>& start_q, 
    const std::array<double, 6>& goal_q,
    double map_width, double map_height, double map_depth,
    double step_size, double neighbor_radius,
    double safety_margin, int max_iter,
    double min_x, double min_y, double min_z
) : map_width(map_width), map_height(map_height), map_depth(map_depth),
    map_min_x(min_x), map_min_y(min_y), map_min_z(min_z),
    step_size(step_size), neighbor_radius(neighbor_radius),
    safety_margin(safety_margin), max_iter(max_iter),
    start_config(start_q), goal_config(goal_q) {
    
    std::cout << "Initializing RRT* path planner with workspace limits:" << std::endl;
    std::cout << "X range: [" << min_x << ", " << min_x + map_width << "]" << std::endl;
    std::cout << "Y range: [" << min_y << ", " << min_y + map_height << "]" << std::endl;
    std::cout << "Z range: [" << min_z << ", " << min_z + map_depth << "]" << std::endl;
    
    // Create collision detector
    collision_detector = std::make_unique<CollisionDetection>(safety_margin);
    
    // Validate start and goal configurations
    if (!collision_detector->isStateValid(start_q) || !collision_detector->isStateValid(goal_q)) {
        throw std::invalid_argument("Start or goal configuration is invalid!");
    }
    
    // Create tree manager
    tree_manager = std::make_unique<TreeManager>(
        start_q, goal_q, map_width, map_height, map_depth,
        step_size, neighbor_radius, min_x, min_y, min_z
    );
    
    // Create path optimizer
    path_optimizer = std::make_unique<PathOptimization>(*collision_detector, step_size);
    
    // Create path planner
    path_planner = std::make_unique<PathPlanner>(
        *tree_manager, *collision_detector, *path_optimizer,
        step_size, neighbor_radius, max_iter
    );
    
    // Visualization disabled for now
    // visualizer = std::make_unique<Visualization>(*collision_detector, true);
    
    // Set goal pose based on goal configuration
    Eigen::Isometry3d goal_pose = RobotKinematics::computeFK(goal_q);
    path_planner->setGoalPose(goal_pose);
}

RRTStarModified::~RRTStarModified() = default;

std::vector<std::shared_ptr<Node>> RRTStarModified::findPath() {
    std::cout << "Starting path planning process..." << std::endl;
    return path_planner->findPath();
}

void RRTStarModified::visualizePath(const std::vector<std::shared_ptr<Node>>& path) {
    // Visualization disabled for now
    std::cout << "Visualization is disabled. Path has " << path.size() << " nodes." << std::endl;
    // This uses the path parameter to avoid unused parameter warnings
}

// void RRTStarModified::exportPathForRobot(
//     const std::vector<std::shared_ptr<Node>>& path, 
//     std::vector<std::array<double, 8>>& robot_commands) {
    
//     path_optimizer->exportPathForRobot(path, robot_commands);
// }

void RRTStarModified::exportPathForRobot(
    const std::vector<std::shared_ptr<Node>>& path, 
    std::vector<std::array<double, 8>>& robot_commands) {
    
    // Call the standalone function from path_export.h
    ::exportPathForRobot(path, robot_commands);
}

// std::vector<std::shared_ptr<Node>> RRTStarModified::createReturnPathSimple(
//     const std::vector<std::shared_ptr<Node>>& forward_path) {
    
//     return tree_manager->createReturnPathSimple(forward_path);
// }

std::vector<std::shared_ptr<Node>> RRTStarModified::createReturnPathSimple(
    const std::vector<std::shared_ptr<Node>>& forward_path) {
    
    // Call the standalone function from path_return.h
    return ::createReturnPathSimple(forward_path);
}

void RRTStarModified::setVisualizationEnabled(bool enabled) {
    // Visualization disabled for now
    std::cout << "Visualization is disabled. Setting to " << (enabled ? "enabled" : "disabled") << " has no effect." << std::endl;
}

void RRTStarModified::setGoalPose(const Eigen::Isometry3d& pose) {
    path_planner->setGoalPose(pose);
}

const Eigen::Isometry3d& RRTStarModified::getGoalPose() const {
    return path_planner->getGoalPose();
}

void RRTStarModified::updateObstacles(const std::vector<Obstacle>& new_obstacles) {
    CollisionDetection::updateObstacles(new_obstacles);
}

const std::vector<Obstacle>& RRTStarModified::getObstacles() {
    return CollisionDetection::getObstacles();
}
