#ifndef COLLISION_DETECTION_H
#define COLLISION_DETECTION_H

#include <array>
#include <vector>
#include <memory>
#include "robot_kinematics.h"
#include "obstacle.h"  // Use existing Obstacle class instead of creating a new one

class CollisionDetection {
private:
    static std::vector<Obstacle> obstacles;  // Use the Obstacle class from obstacle.h
    double safety_margin;

public:
    CollisionDetection(double safety_margin = 0.05);
    ~CollisionDetection() = default;
    
    // Check if a joint configuration is valid
    bool isStateValid(const std::array<double, 6>& q) const;
    
    // Check if a point is inside any obstacle
    bool isObstacle(double x, double y, double z) const;
    
    // Check for line segment intersection with obstacles
    bool lineIntersectsObstacle(
        const std::array<double, 3>& start,
        const std::array<double, 3>& end) const;
    
    // COMPATIBILITY METHOD: Maintain old API for tests
    bool lineAABBIntersection(
        const std::array<double, 3>& start,
        const std::array<double, 3>& end,
        const std::array<double, 3>& box_min,
        const std::array<double, 3>& box_max) const;
    
    // Obstacle management
    static void updateObstacles(const std::vector<Obstacle>& new_obstacles);
    static void addObstacle(const Obstacle& obstacle);
    static void clearObstacles();
    static const std::vector<Obstacle>& getObstacles();
    
    // Set safety margin
    void setSafetyMargin(double margin);
    double getSafetyMargin() const;
};

#endif // COLLISION_DETECTION_H