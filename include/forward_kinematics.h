#ifndef FORWARD_KINEMATICS_H
#define FORWARD_KINEMATICS_H

#include <array>
#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Geometry>

class ForwardKinematics {
public:
    // Define types for 6-DOF joint angles and TCP pose
    using Joint = std::array<double, 6>;
    using Point = std::array<double, 6>; // x,y,z,rx,ry,rz

    // RB5-850 Robot Parameters
    static constexpr double d1 = 169.2;  // mm
    static constexpr double d2 = 148.4;  // mm
    static constexpr double d3 = 148.4;  // mm
    static constexpr double d4 = 110.7;  // mm
    static constexpr double d5 = 110.7;  // mm
    static constexpr double d6 = 96.7;   // mm
    static constexpr double a1 = 425.0;  // mm
    static constexpr double a2 = 392.0;  // mm

    // Constructor
    ForwardKinematics();

    // Calculate forward kinematics
    Point calculateFK(const Joint& joint_angles);

private:
    // Helper functions for transformation matrices (matching IK implementation)
    Eigen::Matrix4d calculateA01(double th1) const;
    Eigen::Matrix4d calculateA12(double th2) const;
    Eigen::Matrix4d calculateA23() const;
    Eigen::Matrix4d calculateA34(double th3) const;
    Eigen::Matrix4d calculateA45() const;
    Eigen::Matrix4d calculateA67() const;
    Eigen::Matrix4d calculateA78(double th5) const;
    Eigen::Matrix4d calculateA89(double th6) const;

    // DH parameters for the robot
    struct DHParams {
        double alpha;  // link twist (rad)
        double a;      // link length (mm)
        double d;      // link offset (mm)
        double theta;  // joint angle (rad)
    };

    // Transformation matrix calculation
    std::array<std::array<double, 4>, 4> calculateTransform(const DHParams& dh);
    
    // Matrix multiplication helper
    std::array<std::array<double, 4>, 4> multiplyMatrices(
        const std::array<std::array<double, 4>, 4>& A,
        const std::array<std::array<double, 4>, 4>& B);

    // Convert rotation matrix to Euler angles
    std::array<double, 3> rotationMatrixToEuler(const std::array<std::array<double, 4>, 4>& T);

    // DH parameters for each joint
    std::array<DHParams, 6> dh_params_;
};

#endif // FORWARD_KINEMATICS_H