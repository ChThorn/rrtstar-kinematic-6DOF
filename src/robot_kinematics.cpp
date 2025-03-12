#include "robot_kinematics.h"
#include <iostream>
#include <stdexcept>

namespace RobotKinematics {
    // Forward kinematics - using the implementation
    Eigen::Isometry3d computeFK(const std::array<double, 6>& q) {
        // Create forward kinematics object
        ForwardKinematics fk;
        
        // Calculate FK
        auto point = fk.calculateFK(q);
        
        // Create Eigen transformation matrix
        Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
        
        // Set translation from point XYZ
        T.translation() = Eigen::Vector3d(point[0], point[1], point[2]);
        
        // Set rotation matrix from Euler angles in point
        Eigen::Matrix3d R;
        R = Eigen::AngleAxisd(point[5], Eigen::Vector3d::UnitZ())
          * Eigen::AngleAxisd(point[4], Eigen::Vector3d::UnitY())
          * Eigen::AngleAxisd(point[3], Eigen::Vector3d::UnitX());
        T.linear() = R;
        
        return T;
    }

    // IK solver for position only
    std::array<double, 6> inverseKinematics(
        const Eigen::Vector3d& target_pos,
        const std::array<double, 6>& q_init,
        double tol,
        int max_iter) {
        
        // Create IK solver
        InverseKinematics ik;
        
        // Call IK solver
        std::vector<IKSolution> solutions = ik.calculateMultipleIKSolutions(
            target_pos[0], target_pos[1], target_pos[2],
            0.0, 0.0, 0.0  // Default orientation
        );
        
        // Use solution evaluator to get best solution
        IKSolutionEvaluator evaluator(q_init);
        
        try {
            // Get best solution
            IKSolution best_solution = evaluator.getBestSolution(solutions);
            return best_solution.joints;
        } catch (const std::runtime_error& e) {
            // If no solution found, return initial configuration
            std::cerr << "IK solution not found: " << e.what() << std::endl;
            return q_init;
        }
    }

    // IK solver with orientation
    std::array<double, 6> inverseKinematicsWithOrientation(
        const Eigen::Isometry3d& target_pose,
        const std::array<double, 6>& q_init,
        double tol,
        int max_iter) {
        
        // Extract position and orientation from target pose
        Eigen::Vector3d position = target_pose.translation();
        Eigen::Matrix3d rotation = target_pose.rotation();
        
        // Convert rotation matrix to Euler angles
        Eigen::Vector3d euler = rotation.eulerAngles(2, 1, 0); // ZYX order
        
        // Create IK solver
        InverseKinematics ik;
        
        // Call IK solver with position and orientation
        std::vector<IKSolution> solutions = ik.calculateMultipleIKSolutions(
            position[0], position[1], position[2],
            euler[2] * 180.0/M_PI, // Convert to degrees (roll - X)
            euler[1] * 180.0/M_PI, // Convert to degrees (pitch - Y)
            euler[0] * 180.0/M_PI  // Convert to degrees (yaw - Z)
        );
        
        // Use solution evaluator to get best solution
        IKSolutionEvaluator evaluator(q_init);
        
        try {
            // Get best solution
            IKSolution best_solution = evaluator.getBestSolution(solutions);
            return best_solution.joints;
        } catch (const std::runtime_error& e) {
            // If no solution found, return initial configuration
            std::cerr << "IK solution with orientation not found: " << e.what() << std::endl;
            return q_init;
        }
    }
}