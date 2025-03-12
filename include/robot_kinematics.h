#ifndef ROBOT_KINEMATICS_H
#define ROBOT_KINEMATICS_H

#include <array>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include "ik_solution_evaluator.h"

// Robot link parameters from kinematics implementation
constexpr double LINK_LENGTHS[] = {
    ForwardKinematics::d1,  // 169.2 mm
    ForwardKinematics::a1,  // 425.0 mm
    ForwardKinematics::a2,  // 392.0 mm
    ForwardKinematics::d4,  // 110.7 mm
    ForwardKinematics::d5,  // 110.7 mm
    ForwardKinematics::d6   // 96.7 mm
};

// Joint limits from InverseKinematics implementation
constexpr std::array<double, 6> JOINT_LIMITS_MIN = {
    InverseKinematics::joint_limits_min[0],  // Base
    InverseKinematics::joint_limits_min[1],  // Shoulder
    InverseKinematics::joint_limits_min[2],  // Elbow
    InverseKinematics::joint_limits_min[3],  // Wrist 1
    InverseKinematics::joint_limits_min[4],  // Wrist 2
    InverseKinematics::joint_limits_min[5]   // Wrist 3
};

constexpr std::array<double, 6> JOINT_LIMITS_MAX = {
    InverseKinematics::joint_limits_max[0],  // Base
    InverseKinematics::joint_limits_max[1],  // Shoulder
    InverseKinematics::joint_limits_max[2],  // Elbow
    InverseKinematics::joint_limits_max[3],  // Wrist 1
    InverseKinematics::joint_limits_max[4],  // Wrist 2
    InverseKinematics::joint_limits_max[5]   // Wrist 3
};

namespace RobotKinematics {
    // Forward kinematics - returns a transformation matrix
    Eigen::Isometry3d computeFK(const std::array<double, 6>& q);
    
    // Numerical IK - position only
    std::array<double, 6> inverseKinematics(
        const Eigen::Vector3d& target_pos,
        const std::array<double, 6>& q_init,
        double tol = 1e-3,
        int max_iter = 100
    );

    // IK with orientation
    std::array<double, 6> inverseKinematicsWithOrientation(
        const Eigen::Isometry3d& target_pose,
        const std::array<double, 6>& q_init,
        double tol = 1e-3,
        int max_iter = 100
    );
    
    // Utility function to wrap angles to [-pi, pi]
    inline double wrapAngle(double angle) {
        return angle - 2*M_PI * std::floor((angle + M_PI) / (2*M_PI));
    }
    
    // Utility function to clamp joint values to limits
    inline double clampToJointLimits(int joint_idx, double value) {
        return std::clamp(value, 
                         JOINT_LIMITS_MIN[joint_idx], 
                         JOINT_LIMITS_MAX[joint_idx]);
    }
}

#endif // ROBOT_KINEMATICS_H