#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "inverse_kinematics.h"
#include <array>

class Obstacle {
public:
    // Constructor to define obstacle properties
    Obstacle(const std::array<double, 3>& position, const std::array<double, 3>& size)
        : position_(position), size_(size) {}

    // Check if a given IK solution collides with this obstacle
    bool isColliding(const IKSolution& solution) const;

private:
    std::array<double, 3> position_; // Position of the obstacle (x, y, z)
    std::array<double, 3> size_;     // Size of the obstacle (width, height, depth)

    // Helper function to calculate the robot's end-effector position
    std::array<double, 3> calculateEndEffectorPosition(const IKSolution& solution) const;
};

#endif // OBSTACLE_H