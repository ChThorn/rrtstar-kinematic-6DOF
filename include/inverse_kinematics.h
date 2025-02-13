#ifndef INVERSE_KINEMATICS_H
#define INVERSE_KINEMATICS_H

#include <Eigen/Dense>
#include <vector>
#include <string>
#include <array>
#include <cmath>

struct RobotConfiguration {
    std::string shoulder; // "LEFTY" or "RIGHTY"
    std::string elbow;    // "UP" or "DOWN"
    std::string wrist;    // "FLIP" or "NOFLIP"
};

struct IKSolution {
    std::array<double, 6> joints;
    RobotConfiguration configuration;
};

class InverseKinematics {
public:
    // Constants
    static constexpr double D2R = M_PI / 180.0;
    static constexpr double R2D = 180.0 / M_PI;

    // Link parameters (RB5-850)
    static constexpr double d1 = 169.2;
    static constexpr double d2 = 148.4;
    static constexpr double d3 = 148.4;
    static constexpr double d4 = 110.7;
    static constexpr double d5 = 110.7;
    static constexpr double d6 = 96.7;
    static constexpr double a1 = 425.0;
    static constexpr double a2 = 392.0;

    // Joint limits
    static constexpr double joint_limits_min[6] = {
        -2 * M_PI,  // Joint 1: Base (-360°)
        -2.059,     // Joint 2: Shoulder (-118°)
        -3.927,     // Joint 3: Elbow (-225°)
        -2 * M_PI,  // Joint 4: Wrist 1 (-360°)
        -1.693,     // Joint 5: Wrist 2 (-97°)
        -2 * M_PI   // Joint 6: Wrist 3 (-360°)
    };

    static constexpr double joint_limits_max[6] = {
        2 * M_PI,   // Joint 1: Base (+360°)
        2.094,      // Joint 2: Shoulder (+120°)
        0.192,      // Joint 3: Elbow (+11°)
        2 * M_PI,   // Joint 4: Wrist 1 (+360°)
        M_PI,       // Joint 5: Wrist 2 (+180°)
        2 * M_PI    // Joint 6: Wrist 3 (+360°)
    };

    // Main calculation function
    std::vector<IKSolution> calculateMultipleIKSolutions(
        double input_x, double input_y, double input_z,
        double input_rx, double input_ry, double input_rz);

    // Validate joint angles
    bool isValidJointAngle(int joint_index, double angle) const;

    // Helper function for rotation matrix
    Eigen::Matrix3d calculateRotationMatrix(double rx, double ry, double rz) const;

private:
    // Helper functions for transformation matrices
    Eigen::Matrix4d calculateA01(double th1) const;
    Eigen::Matrix4d calculateA12(double th2) const;
    Eigen::Matrix4d calculateA23() const;
    Eigen::Matrix4d calculateA34(double th3) const;
    Eigen::Matrix4d calculateA45() const;
    Eigen::Matrix4d calculateA67() const;
    Eigen::Matrix4d calculateA78(double th5) const;
    Eigen::Matrix4d calculateA89(double th6) const;

    double safe_acos(double x) const;
    double safe_asin(double x) const;

    // Workspace boundaries (adjusted values; verify based on your robot)
    static constexpr double workspace_min_x = -1500.0; // mm
    static constexpr double workspace_max_x = 1500.0;  // mm
    static constexpr double workspace_min_y = -1500.0; // mm
    static constexpr double workspace_max_y = 1500.0;  // mm
    static constexpr double workspace_min_z = -500.0;  // mm
    static constexpr double workspace_max_z = 2000.0;  // mm

    

    
};

#endif // INVERSE_KINEMATICS_H