#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <array>
#include <vector>
#include <memory>
#include "robot_kinematics.h"
#include "ik_solution_evaluator.h"

// Define planning obstacle structure
struct PlanningObstacle {
    std::array<double, 3> min_point;
    std::array<double, 3> max_point;
    
    PlanningObstacle(const std::array<double, 3>& min, const std::array<double, 3>& max)
        : min_point(min), max_point(max) {}
};

class CollisionDetection {
private:
    static std::vector<PlanningObstacle> obstacles;
    double safety_margin;

public:
    CollisionDetection(double safety_margin = 0.05);
    ~CollisionDetection() = default;
    
    // Check if a joint configuration is valid
    bool isStateValid(const std::array<double, 6>& q) const;
    
    // Check if a point is inside any obstacle
    bool isObstacle(double x, double y, double z) const;
    
    // Check for line segment intersection with AABB
    bool lineAABBIntersection(
        const std::array<double, 3>& start,
        const std::array<double, 3>& end,
        const std::array<double, 3>& box_min,
        const std::array<double, 3>& box_max) const;
    
    // Obstacle management
    static void updateObstacles(const std::vector<PlanningObstacle>& new_obstacles);
    static void addObstacle(const PlanningObstacle& obstacle);
    static void clearObstacles();
    static const std::vector<PlanningObstacle>& getObstacles();
    
    // Set safety margin
    void setSafetyMargin(double margin);
    double getSafetyMargin() const;
};

#endif // COLLISION_DETECTION_H