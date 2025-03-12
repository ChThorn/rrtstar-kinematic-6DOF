#include "collision_detection.h"
#include <cmath>
#include <algorithm>

// Initialize static obstacle list using the factory method from Obstacle class
std::vector<Obstacle> CollisionDetection::obstacles = Obstacle::createDefaultObstacles();

CollisionDetection::CollisionDetection(double safety_margin)
    : safety_margin(safety_margin) {
}

bool CollisionDetection::isStateValid(const std::array<double, 6>& q) const {
    // 1. Check joint limits
    for (int j = 0; j < 6; ++j) {
        if (q[j] < JOINT_LIMITS_MIN[j] || q[j] > JOINT_LIMITS_MAX[j]) {
            return false;
        }
    }
    
    // 2. Use the Obstacle class directly for collision checking
    IKSolution solution;
    solution.joints = q;
    solution.configuration = {"UNKNOWN", "UNKNOWN", "UNKNOWN"}; // Configuration doesn't matter for collision check
    
    // Check each obstacle for collision
    for (const auto& obstacle : obstacles) {
        if (obstacle.isColliding(solution)) {
            return false; // Collision detected
        }
    }
    
    return true; // No collision
}

bool CollisionDetection::isObstacle(double x, double y, double z) const {
    // For each obstacle, check if the point is inside
    std::array<double, 3> point = {x, y, z};
    
    for (const auto& obstacle : obstacles) {
        // Extract position and size from obstacle
        const auto& center = obstacle.getPosition();
        const auto& size = obstacle.getSize();
        
        // Check if point is inside the obstacle (plus safety margin)
        bool inside = true;
        for (int i = 0; i < 3; ++i) {
            double min_val = center[i] - (size[i] / 2.0) - safety_margin;
            double max_val = center[i] + (size[i] / 2.0) + safety_margin;
            
            if (point[i] < min_val || point[i] > max_val) {
                inside = false;
                break;
            }
        }
        
        if (inside) return true; // Point is inside an obstacle
    }
    
    return false; // Point is not inside any obstacle
}

bool CollisionDetection::lineIntersectsObstacle(
    const std::array<double, 3>& start,
    const std::array<double, 3>& end) const {
    
    // Check each obstacle for line intersection
    for (const auto& obstacle : obstacles) {
        // Extract position and size from obstacle
        const auto& center = obstacle.getPosition();
        const auto& size = obstacle.getSize();
        
        // Calculate the AABB min and max points
        std::array<double, 3> box_min = {
            center[0] - (size[0] / 2.0) - safety_margin,
            center[1] - (size[1] / 2.0) - safety_margin,
            center[2] - (size[2] / 2.0) - safety_margin
        };
        
        std::array<double, 3> box_max = {
            center[0] + (size[0] / 2.0) + safety_margin,
            center[1] + (size[1] / 2.0) + safety_margin,
            center[2] + (size[2] / 2.0) + safety_margin
        };
        
        // Check for line-AABB intersection using the compatibility method
        if (lineAABBIntersection(start, end, box_min, box_max)) {
            return true; // Line intersects this obstacle
        }
    }
    
    return false; // No intersection with any obstacle
}

// COMPATIBILITY METHOD: Maintain old API for tests
bool CollisionDetection::lineAABBIntersection(
    const std::array<double, 3>& start,
    const std::array<double, 3>& end,
    const std::array<double, 3>& box_min,
    const std::array<double, 3>& box_max) const {
    
    double tmin = 0.0;
    double tmax = 1.0;
    std::array<double, 3> dir = {
        end[0] - start[0], 
        end[1] - start[1], 
        end[2] - start[2]
    };
    
    for (int i = 0; i < 3; ++i) {
        if (std::abs(dir[i]) < 1e-9) {
            if (start[i] < box_min[i] || start[i] > box_max[i]) {
                return false;
            }
        } else {
            double ood = 1.0 / dir[i];
            double t1 = (box_min[i] - start[i]) * ood;
            double t2 = (box_max[i] - start[i]) * ood;
            if (t1 > t2) std::swap(t1, t2);
            tmin = std::max(tmin, t1);
            tmax = std::min(tmax, t2);
            if (tmin > tmax) return false;
        }
    }
    return tmin <= 1.0 && tmax >= 0.0;
}

void CollisionDetection::updateObstacles(const std::vector<Obstacle>& new_obstacles) {
    obstacles = new_obstacles;
}

void CollisionDetection::addObstacle(const Obstacle& obstacle) {
    obstacles.push_back(obstacle);
}

void CollisionDetection::clearObstacles() {
    obstacles.clear();
}

const std::vector<Obstacle>& CollisionDetection::getObstacles() {
    return obstacles;
}

void CollisionDetection::setSafetyMargin(double margin) {
    safety_margin = margin;
}

double CollisionDetection::getSafetyMargin() const {
    return safety_margin;
}