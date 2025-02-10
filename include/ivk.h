#ifndef IVK_H
#define IVK_H

#include <Eigen/Dense>
#include <vector>
#include <string>
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

class Ivk {
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

    // Main calculation function
    std::vector<IKSolution> calculateMultipleIKSolutions(
        double input_x, double input_y, double input_z,
        double input_rx, double input_ry, double input_rz);

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
    
    // Helper function for rotation matrix
    Eigen::Matrix3d calculateRotationMatrix(double rx, double ry, double rz) const;
};

#endif // IVK_H