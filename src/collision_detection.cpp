#include "collision_detection.h"
#include <cmath>
#include <algorithm>

// Initialize static obstacle list
std::vector<PlanningObstacle> CollisionDetection::obstacles = {
    PlanningObstacle({2, 2, 2}, {4, 4, 4}),  // Near the start
    PlanningObstacle({6, 6, 6}, {8, 8, 8})   // Near the goal
};

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
    
    // 2. Convert planning obstacles to IK solution evaluator obstacles
    std::vector<Obstacle> ik_obstacles;
    for (const auto& plan_obstacle : obstacles) {
        // Center position
        std::array<double, 3> center = {
            (plan_obstacle.min_point[0] + plan_obstacle.max_point[0]) / 2.0,
            (plan_obstacle.min_point[1] + plan_obstacle.max_point[1]) / 2.0,
            (plan_obstacle.min_point[2] + plan_obstacle.max_point[2]) / 2.0
        };
        
        // Size
        std::array<double, 3> size = {
            plan_obstacle.max_point[0] - plan_obstacle.min_point[0],
            plan_obstacle.max_point[1] - plan_obstacle.min_point[1],
            plan_obstacle.max_point[2] - plan_obstacle.min_point[2]
        };
        
        ik_obstacles.push_back(Obstacle(center, size));
    }
    
    // 3. Use IKSolutionEvaluator to check for collisions
    IKSolution solution;
    solution.joints = q;
    solution.configuration = {"UNKNOWN", "UNKNOWN", "UNKNOWN"}; // Configuration doesn't matter for collision check
    
    IKSolutionEvaluator evaluator(q, ik_obstacles);
    double collision_score = evaluator.getCollisionScore(solution);
    
    return collision_score > 0.0; // Non-zero score means no collision
}

bool CollisionDetection::isObstacle(double x, double y, double z) const {
    std::array<double, 3> point = {x, y, z};
    for (const auto& obstacle : obstacles) {
        bool inside = true;
        for (int i = 0; i < 3; ++i) {
            if (point[i] < obstacle.min_point[i] - safety_margin ||
                point[i] > obstacle.max_point[i] + safety_margin) {
                inside = false;
                break;
            }
        }
        if (inside) return true;
    }
    return false;
}

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

void CollisionDetection::updateObstacles(const std::vector<PlanningObstacle>& new_obstacles) {
    obstacles = new_obstacles;
}

void CollisionDetection::addObstacle(const PlanningObstacle& obstacle) {
    obstacles.push_back(obstacle);
}

void CollisionDetection::clearObstacles() {
    obstacles.clear();
}

const std::vector<PlanningObstacle>& CollisionDetection::getObstacles() {
    return obstacles;
}

void CollisionDetection::setSafetyMargin(double margin) {
    safety_margin = margin;
}

double CollisionDetection::getSafetyMargin() const {
    return safety_margin;
}