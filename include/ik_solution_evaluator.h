#ifndef IK_SOLUTION_EVALUATOR_H
#define IK_SOLUTION_EVALUATOR_H

#include "inverse_kinematics.h"
#include "obstacle.h"
#include <array>
#include <vector>
#include <string>
#include <cmath>
#include <limits>

class IKSolutionEvaluator {
public:
    struct SolutionScore {
        IKSolution solution;
        double score;

        // Constructor for easy sorting
        SolutionScore(const IKSolution& sol, double s) 
            : solution(sol), score(s) {}
    };

    // Constructor that takes current joint positions and obstacles
    IKSolutionEvaluator(const std::array<double, 6>& current_joints, const std::vector<Obstacle>& obstacles = {});

    // Conversion constants
    static constexpr double D2R = M_PI / 180.0;
    static constexpr double R2D = 180.0 / M_PI;

    // Public utility functions
    double normalizeAngle(double angle) const;
    double getAngularDistance(double angle1, double angle2) const;

    // Main evaluation function
    IKSolution getBestSolution(const std::vector<IKSolution>& solutions);

    // Getter methods for evaluation functions
    double getJointDistanceScore(const std::array<double, 6>& solution) const {
        return evaluateJointDistance(solution);
    }
    double getJointLimitsScore(const std::array<double, 6>& solution) const {
        return evaluateJointLimits(solution);
    }
    double getManipulabilityScore(const std::array<double, 6>& solution) const {
        return evaluateManipulability(solution);
    }
    double getConfigurationScore(const RobotConfiguration& config) const {
        return evaluateConfiguration(config);
    }
    double getCollisionScore(const IKSolution& solution) const {
        return evaluateCollision(solution);
    }

    // Total movement calculation
    double getTotalMovement(const IKSolution& solution) const {
        return getTotalMovement(solution.joints);
    }
    double getTotalMovement(const std::array<double, 6>& joint_angles) const;

private:
    // Current joint positions
    std::array<double, 6> current_joints_;

    // List of obstacles
    std::vector<Obstacle> obstacles_;

    // Evaluation metrics
    double evaluateJointDistance(const std::array<double, 6>& solution) const;
    double evaluateJointLimits(const std::array<double, 6>& solution) const;
    double evaluateManipulability(const std::array<double, 6>& solution) const;
    double evaluateConfiguration(const RobotConfiguration& config) const;
    double evaluateCollision(const IKSolution& solution) const;

    // Weights for each metric
    static constexpr double JOINT_DISTANCE_WEIGHT = 0.7;
    static constexpr double JOINT_LIMITS_WEIGHT = 0.15;
    static constexpr double MANIPULABILITY_WEIGHT = 0.1;
    static constexpr double CONFIGURATION_WEIGHT = 0.05;
    static constexpr double COLLISION_WEIGHT = 0.05;

    static constexpr double JOINT_DISTANCE_THRESHOLD = 5.0; // degrees
};

#endif // IK_SOLUTION_EVALUATOR_H