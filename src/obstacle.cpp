#include "obstacle.h"
#include <cmath>

// Calculate the end-effector position based on joint angles
std::array<double, 3> Obstacle::calculateEndEffectorPosition(const IKSolution& solution) const {
    // Example placeholder values for the end-effector position
    // Replace these with actual calculations when forward kinematics is implemented
    double x = solution.joints[0] * 10.0; // Example scaling factor
    double y = solution.joints[1] * 10.0;
    double z = solution.joints[2] * 10.0;

    return {x, y, z};
}

// Check if the solution collides with the obstacle
bool Obstacle::isColliding(const IKSolution& solution) const {
    std::array<double, 3> end_effector_position = calculateEndEffectorPosition(solution);

    // Check if the end-effector is within the bounding box of the obstacle
    for (size_t i = 0; i < 3; ++i) {
        double lower_bound = position_[i] - size_[i] / 2.0;
        double upper_bound = position_[i] + size_[i] / 2.0;

        if (end_effector_position[i] < lower_bound || end_effector_position[i] > upper_bound) {
            return false; // No collision
        }
    }

    return true; // Collision detected
}