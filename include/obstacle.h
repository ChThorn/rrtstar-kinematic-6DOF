// #ifndef OBSTACLE_H
// #define OBSTACLE_H

// #include "inverse_kinematics.h"
// #include <array>

// class Obstacle {
// public:
//     // Constructor to define obstacle properties
//     Obstacle(const std::array<double, 3>& position, const std::array<double, 3>& size)
//         : position_(position), size_(size) {}

//     // Check if a given IK solution collides with this obstacle
//     bool isColliding(const IKSolution& solution) const;

// private:
//     std::array<double, 3> position_; // Position of the obstacle (x, y, z)
//     std::array<double, 3> size_;     // Size of the obstacle (width, height, depth)

//     // Helper function to calculate the robot's end-effector position
//     std::array<double, 3> calculateEndEffectorPosition(const IKSolution& solution) const;
// };

// #endif // OBSTACLE_H


// obstacle.h
#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "inverse_kinematics.h"
#include <array>
#include <opencv2/core.hpp>

class Obstacle {
public:
    // Constructor for static obstacles
    Obstacle(const std::array<double, 3>& position, const std::array<double, 3>& size)
        : position_(position), size_(size), is_dynamic_(false) {}

    // Constructor for dynamic obstacles
    Obstacle(bool is_dynamic = true)
        : is_dynamic_(is_dynamic) {}

    // Update obstacle position and size based on depth and ROI
    void updateFromDepthAndROI(const cv::Rect& roi, float depth_value) {
        if (!is_dynamic_) return; // Only update if the obstacle is dynamic

        // Example: Use ROI center and depth to define obstacle position
        position_[0] = roi.x + roi.width / 2.0;  // X center of ROI
        position_[1] = roi.y + roi.height / 2.0; // Y center of ROI
        position_[2] = depth_value;              // Z position based on depth

        // Example: Define obstacle size proportional to ROI dimensions
        size_[0] = roi.width;   // Width of ROI
        size_[1] = roi.height;  // Height of ROI
        size_[2] = 50.0;        // Fixed depth size (adjust as needed)
    }

    // Check if a given IK solution collides with this obstacle
    bool isColliding(const IKSolution& solution) const;

private:
    std::array<double, 3> position_; // Position of the obstacle (x, y, z)
    std::array<double, 3> size_;     // Size of the obstacle (width, height, depth)
    bool is_dynamic_;                // Flag to indicate if the obstacle is dynamic

    // Helper function to calculate the robot's end-effector position
    std::array<double, 3> calculateEndEffectorPosition(const IKSolution& solution) const;
};

#endif // OBSTACLE_H