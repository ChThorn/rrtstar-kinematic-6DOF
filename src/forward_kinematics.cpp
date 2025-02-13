#include "forward_kinematics.h"
#include <iostream>

ForwardKinematics::ForwardKinematics() {}

Eigen::Matrix4d ForwardKinematics::calculateA01(double th1) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th1), -cos(-half_pi) * sin(th1), sin(-half_pi) * sin(th1), 0,
         sin(th1), cos(-half_pi) * cos(th1), -sin(-half_pi) * cos(th1), 0,
         0, sin(-half_pi), cos(-half_pi), d1,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA12(double th2) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th2 - half_pi), -sin(th2 - half_pi), 0, 0,
         sin(th2 - half_pi), cos(th2 - half_pi), 0, 0,
         0, 0, 1, -d2,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA23() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a1,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA34(double th3) const {
    Eigen::Matrix4d A;
    A << cos(th3), -sin(th3), 0, 0,
         sin(th3), cos(th3), 0, 0,
         0, 0, 1, d3,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA45() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a2,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA67() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
         0, 0, -1, 0,
         0, 1, 0, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA78(double th5) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th5), -cos(-half_pi) * sin(th5), sin(-half_pi) * sin(th5), 0,
         sin(th5), cos(-half_pi) * cos(th5), -sin(-half_pi) * cos(th5), 0,
         0, sin(-half_pi), cos(-half_pi), d5,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d ForwardKinematics::calculateA89(double th6) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th6), -cos(half_pi) * sin(th6), sin(half_pi) * sin(th6), 0,
         sin(th6), cos(half_pi) * cos(th6), -sin(half_pi) * cos(th6), 0,
         0, sin(half_pi), cos(half_pi), -d6,
         0, 0, 0, 1;
    return A;
}

ForwardKinematics::Point ForwardKinematics::calculateFK(const Joint& joint_angles_deg) {
    // Convert to radians
    Joint angles_rad;
    for(int i = 0; i < 6; i++) {
        angles_rad[i] = joint_angles_deg[i] * M_PI / 180.0;
    }

    // Match IK's transformation sequence
    auto A01 = calculateA01(angles_rad[0]);
    auto A12 = calculateA12(angles_rad[1]);
    auto A23 = calculateA23();
    auto A34 = calculateA34(angles_rad[2]);
    auto A45 = calculateA45();
    auto A67 = calculateA67();
    auto A78 = calculateA78(angles_rad[4]);
    auto A89 = calculateA89(angles_rad[5]);

    // Calculate total transformation
    auto T = A01 * A12 * A23 * A34 * A45 * A67 * A78 * A89;

    Point result;
    // Position
    result[0] = T(0,3);
    result[1] = T(1,3);
    result[2] = T(2,3);

    // Extract Euler angles using same convention as IK
    Eigen::Matrix3d R = T.block<3,3>(0,0);
    Eigen::Vector3d euler = R.eulerAngles(2,1,0); // ZYX order

    result[3] = euler[2];  // roll  (X)
    result[4] = euler[1];  // pitch (Y)
    result[5] = euler[0];  // yaw   (Z)

    return result;
}

std::array<std::array<double, 4>, 4> ForwardKinematics::calculateTransform(const DHParams& dh) {
    double ct = cos(dh.theta);
    double st = sin(dh.theta);
    double ca = cos(dh.alpha);
    double sa = sin(dh.alpha);

    return {{
        {ct,     -st*ca,  st*sa,   dh.a*ct},
        {st,     ct*ca,   -ct*sa,  dh.a*st},
        {0,      sa,      ca,      dh.d   },
        {0,      0,       0,       1      }
    }};
}

std::array<std::array<double, 4>, 4> ForwardKinematics::multiplyMatrices(
    const std::array<std::array<double, 4>, 4>& A,
    const std::array<std::array<double, 4>, 4>& B) {
    
    std::array<std::array<double, 4>, 4> C = {{
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0},
        {0, 0, 0, 0}
    }};

    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            for (int k = 0; k < 4; ++k) {
                C[i][j] += A[i][k] * B[k][j];
            }
        }
    }

    return C;
}

std::array<double, 3> ForwardKinematics::rotationMatrixToEuler(
    const std::array<std::array<double, 4>, 4>& T) {
    
    std::array<double, 3> euler;
    
    // Calculate RX (roll)
    euler[0] = atan2(T[2][1], T[2][2]);
    
    // Calculate RY (pitch)
    double sy = -T[2][0];
    euler[1] = atan2(sy, sqrt(1 - sy*sy));
    
    // Calculate RZ (yaw)
    euler[2] = atan2(T[1][0], T[0][0]);
    
    return euler;
}