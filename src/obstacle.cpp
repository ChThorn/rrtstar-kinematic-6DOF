#include "obstacle.h"

std::array<double, 3> Obstacle::calculateEndEffectorPosition(const IKSolution& solution) const {
    // Replace with actual forward kinematics calculation
    double x = solution.joints[0] * 10.0; // Example scaling factor
    double y = solution.joints[1] * 10.0;
    double z = solution.joints[2] * 10.0;

    return {x, y, z};
}

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

void Obstacle::updateFromDepthAndROI(const cv::Rect& roi, float depth_value) {
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

// Factory method for creating a box obstacle
Obstacle Obstacle::createBox(const std::array<double, 3>& center, const std::array<double, 3>& dimensions) {
    return Obstacle(center, dimensions);
}

// Factory method for creating a cylindrical obstacle (approximated as a box)
Obstacle Obstacle::createCylinder(double x, double y, double z, double radius, double height) {
    std::array<double, 3> center = {x, y, z};
    std::array<double, 3> dimensions = {radius * 2.0, radius * 2.0, height};
    return Obstacle(center, dimensions);
}

// Factory method for creating a spherical obstacle (approximated as a box)
Obstacle Obstacle::createSphere(double x, double y, double z, double radius) {
    std::array<double, 3> center = {x, y, z};
    std::array<double, 3> dimensions = {radius * 2.0, radius * 2.0, radius * 2.0};
    return Obstacle(center, dimensions);
}

// Factory method for creating a standard set of obstacles for planning
std::vector<Obstacle> Obstacle::createDefaultObstacles() {
    std::vector<Obstacle> obstacles;
    
    // Add some default obstacles for testing and development
    obstacles.push_back(createBox({3.0, 3.0, 3.0}, {2.0, 2.0, 2.0}));   // Small box near start
    obstacles.push_back(createBox({7.0, 7.0, 7.0}, {2.0, 2.0, 2.0}));   // Small box near goal
    obstacles.push_back(createCylinder(5.0, 5.0, 0.0, 1.0, 3.0));      // Cylinder in the middle
    
    return obstacles;
}