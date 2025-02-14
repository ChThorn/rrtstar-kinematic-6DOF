#include "ik_solution_evaluator.h"
#include <iostream>
#include <algorithm>

IKSolutionEvaluator::IKSolutionEvaluator(const std::array<double, 6>& current_joints)
    : current_joints_(current_joints) {}

IKSolution IKSolutionEvaluator::getBestSolution(const std::vector<IKSolution>& solutions) {
    if (solutions.empty()) {
        throw std::runtime_error("No IK solutions provided");
    }

    std::vector<SolutionScore> scored_solutions;
    scored_solutions.reserve(solutions.size());

    // Filter out invalid solutions that violate joint limits
    std::vector<IKSolution> valid_solutions;
    for (const auto& solution : solutions) {
        bool is_valid = true;
        for (size_t i = 0; i < 6; ++i) {
            double angle_rad = solution.joints[i] * D2R;
            if (angle_rad < InverseKinematics::joint_limits_min[i] ||
                angle_rad > InverseKinematics::joint_limits_max[i]) {
                is_valid = false;
                break;
            }
        }
        if (is_valid) {
            valid_solutions.push_back(solution);
        }
    }

    if (valid_solutions.empty()) {
        throw std::runtime_error("No valid IK solutions within joint limits");
    }

    // Score each valid solution
    for (const auto& solution : valid_solutions) {
        double total_score = 0.0;

        // Calculate individual metrics
        double distance_score = evaluateJointDistance(solution.joints);
        double limits_score = evaluateJointLimits(solution.joints);
        double manipulability_score = evaluateManipulability(solution.joints);
        double config_score = evaluateConfiguration(solution.configuration);

        // Combine scores with weights
        total_score = JOINT_DISTANCE_WEIGHT * distance_score +
                      JOINT_LIMITS_WEIGHT * limits_score +
                      MANIPULABILITY_WEIGHT * manipulability_score +
                      CONFIGURATION_WEIGHT * config_score;

        // Debugging output
        std::cout << "Solution joints: ";
        for (double j : solution.joints) std::cout << j << " ";
        std::cout << "\nDistance Score: " << distance_score
                  << "\nLimits Score: " << limits_score
                  << "\nManipulability Score: " << manipulability_score
                  << "\nConfiguration Score: " << config_score
                  << "\nTotal Score: " << total_score << std::endl;

        scored_solutions.emplace_back(solution, total_score);
    }

    // Find solution with highest score
    auto best_solution = std::max_element(
        scored_solutions.begin(),
        scored_solutions.end(),
        [](const SolutionScore& a, const SolutionScore& b) {
            return a.score < b.score;
        }
    );

    return best_solution->solution;
}

double IKSolutionEvaluator::evaluateJointDistance(const std::array<double, 6>& solution) const {
    double score = 1.0;
    double max_movement = 0.0;
    double total_movement = 0.0;

    for (size_t i = 0; i < 6; ++i) {
        double diff = getAngularDistance(solution[i], current_joints_[i]);
        max_movement = std::max(max_movement, diff);
        total_movement += diff;
    }

    // Penalize based on maximum single joint movement
    score *= 1.0 / (1.0 + max_movement / 45.0);  // Softer penalty, 45° reference
    // Penalize based on total movement
    score *= 1.0 / (1.0 + total_movement / 180.0);  // Using 180° as reference
    return score;
}

double IKSolutionEvaluator::evaluateConfiguration(const RobotConfiguration& config) const {
    double score = 1.0;
    if (config.shoulder == "RIGHTY") score *= 1.2;  // Reduced from 3.0
    if (config.elbow == "UP") score *= 1.5;        // Reduced from 10.0
    if (config.wrist == "NOFLIP") score *= 1.1;    // Reduced from 1.5
    return score;
}

double IKSolutionEvaluator::evaluateJointLimits(const std::array<double, 6>& solution) const {
    double score = 0.0;

    for (size_t i = 0; i < 6; ++i) {
        double angle_rad = solution[i] * D2R; // Convert to radians for limit check
        double min = InverseKinematics::joint_limits_min[i];
        double max = InverseKinematics::joint_limits_max[i];

        if (angle_rad < min || angle_rad > max) {
            return 0.0; // Immediate rejection for out-of-bounds solutions
        }

        double mid = (min + max) / 2.0;
        double range = max - min;
        double normalized_dist = 2.0 * std::abs(angle_rad - mid) / range;
        score += 1.0 - std::min(normalized_dist, 1.0);
    }

    return score / 6.0;
}

double IKSolutionEvaluator::evaluateManipulability(const std::array<double, 6>& solution) const {
    double score = 1.0;

    // Penalize near-singular configurations
    for (size_t i = 0; i < 5; ++i) {
        double angle_diff = std::abs(normalizeAngle(solution[i] - solution[i + 1]));
        if (angle_diff < 0.1) { // Near-singular threshold
            score *= 0.1; // Heavily penalize
        }
    }

    return score;
}

double IKSolutionEvaluator::getAngularDistance(double angle1, double angle2) const {
    // Normalize angles to -180 to 180 range
    angle1 = normalizeAngle(angle1);
    angle2 = normalizeAngle(angle2);
    double diff = std::abs(angle1 - angle2);
    return std::min(diff, 360.0 - diff);
}

double IKSolutionEvaluator::normalizeAngle(double angle) const {
    angle = std::fmod(angle + 180.0, 360.0);
    if (angle < 0) angle += 360.0;
    return angle - 180.0;
}

double IKSolutionEvaluator::getTotalMovement(const std::array<double, 6>& joint_angles) const {
    double total_movement = 0.0;
    for (size_t i = 0; i < 6; ++i) {
        total_movement += getAngularDistance(joint_angles[i], current_joints_[i]);
    }
    return total_movement;
}