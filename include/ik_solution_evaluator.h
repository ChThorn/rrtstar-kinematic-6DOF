#ifndef IK_SOLUTION_EVALUATOR_H
#define IK_SOLUTION_EVALUATOR_H

#include "inverse_kinematics.h"
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
        SolutionScore(const IKSolution& sol, double s) : solution(sol), score(s) {}
    };

    // Constructor that takes current joint positions
    IKSolutionEvaluator(const std::array<double, 6>& current_joints);

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

    // Getter methods for weights
    static constexpr double getJointDistanceWeight() { return JOINT_DISTANCE_WEIGHT; }
    static constexpr double getJointLimitsWeight() { return JOINT_LIMITS_WEIGHT; }
    static constexpr double getManipulabilityWeight() { return MANIPULABILITY_WEIGHT; }
    static constexpr double getConfigurationWeight() { return CONFIGURATION_WEIGHT; }

    // Total movement calculation
    double getTotalMovement(const IKSolution& solution) const {
        return getTotalMovement(solution.joints);
    }
    double getTotalMovement(const std::array<double, 6>& joint_angles) const;

private:
    // Current joint positions
    std::array<double, 6> current_joints_;

    // Evaluation metrics
    double evaluateJointDistance(const std::array<double, 6>& solution) const;
    double evaluateJointLimits(const std::array<double, 6>& solution) const;
    double evaluateManipulability(const std::array<double, 6>& solution) const;
    double evaluateConfiguration(const RobotConfiguration& config) const;

    // Weights for each metric (configurable via constructor in future)
    static constexpr double JOINT_DISTANCE_WEIGHT = 0.7;
    static constexpr double JOINT_LIMITS_WEIGHT = 0.15;
    static constexpr double MANIPULABILITY_WEIGHT = 0.1;
    static constexpr double CONFIGURATION_WEIGHT = 0.05;

    // Thresholds for scoring
    static constexpr double JOINT_DISTANCE_THRESHOLD = 5.0; // degrees
};

#endif // IK_SOLUTION_EVALUATOR_H