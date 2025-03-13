#ifndef PATH_OPTIMIZATION_H
#define PATH_OPTIMIZATION_H

#include <vector>
#include <array>
#include <memory>
#include <Eigen/Dense>
#include "tree_management.h"
#include "collision_detection.h"

// Path quality metrics structure
struct PathQualityMetrics {
    double total_length;
    double max_step;
    double avg_step;
    double smoothness;

    PathQualityMetrics() : total_length(0.0), max_step(0.0), avg_step(0.0), smoothness(0.0) {}
};

class PathOptimization {
public:
    // Robot motion parameters - public for other components to use
    static constexpr double MAX_JOINT_VEL = M_PI/4;  // rad/s (45 deg/s)
    static constexpr double CONTROL_RATE = 100.0;    // Hz

private:
    CollisionDetection& collision_detector;
    double step_size;

public:
    PathOptimization(CollisionDetection& collision_detector, double step_size);
    ~PathOptimization() = default;
    
    // Basic path optimization
    void optimizePath(std::vector<std::shared_ptr<Node>>& path);
    
    // Advanced path smoothing methods
    void refinePathDynamically(std::vector<std::shared_ptr<Node>>& path);
    void refinePathWithIK(std::vector<std::shared_ptr<Node>>& path);
    void applyQuinticSplineWithConstraints(std::vector<std::shared_ptr<Node>>& path);
    
    // Utility functions
    void applyCubicSpline(const std::vector<std::array<double, 3>>& points);
    PathQualityMetrics evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path);
    
    // Path export for robot
    // void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path,
    //                         std::vector<std::array<double, 8>>& robot_commands);
    
    // Helper function to check collision between nodes
    std::pair<bool, int> isCollisionFree(std::shared_ptr<Node> node1, std::shared_ptr<Node> node2);
};

#endif // PATH_OPTIMIZATION_H