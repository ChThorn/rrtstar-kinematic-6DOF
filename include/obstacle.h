#ifndef OBSTACLE_H
#define OBSTACLE_H

#include "inverse_kinematics.h"
#include <array>
#include <vector>
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
    void updateFromDepthAndROI(const cv::Rect& roi, float depth_value);

    // Check if a given IK solution collides with this obstacle
    bool isColliding(const IKSolution& solution) const;

    // Getter methods for position and size
    const std::array<double, 3>& getPosition() const { return position_; }
    const std::array<double, 3>& getSize() const { return size_; }
    bool isDynamic() const { return is_dynamic_; }

    // Factory methods for standard obstacle types
    static Obstacle createBox(const std::array<double, 3>& center, const std::array<double, 3>& dimensions);
    static Obstacle createCylinder(double x, double y, double z, double radius, double height);
    static Obstacle createSphere(double x, double y, double z, double radius);
    
    // Factory method for creating a set of default obstacles for planning
    static std::vector<Obstacle> createDefaultObstacles();

private:
    std::array<double, 3> position_; // Position of the obstacle (x, y, z)
    std::array<double, 3> size_;     // Size of the obstacle (width, height, depth)
    bool is_dynamic_;                // Flag to indicate if the obstacle is dynamic

    // Helper function to calculate the robot's end-effector position
    std::array<double, 3> calculateEndEffectorPosition(const IKSolution& solution) const;
};

#endif // OBSTACLE_H