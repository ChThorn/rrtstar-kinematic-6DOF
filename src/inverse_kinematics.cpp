#include "inverse_kinematics.h"
#include <iostream>
#include <algorithm>

// Transformation matrices
Eigen::Matrix4d InverseKinematics::calculateA01(double th1) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th1), -cos(-half_pi) * sin(th1), sin(-half_pi) * sin(th1), 0,
         sin(th1), cos(-half_pi) * cos(th1), -sin(-half_pi) * cos(th1), 0,
         0, sin(-half_pi), cos(-half_pi), d1,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA12(double th2) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th2 - half_pi), -sin(th2 - half_pi), 0, 0,
         sin(th2 - half_pi), cos(th2 - half_pi), 0, 0,
         0, 0, 1, -d2,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA23() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a1,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA34(double th3) const {
    Eigen::Matrix4d A;
    A << cos(th3), -sin(th3), 0, 0,
         sin(th3), cos(th3), 0, 0,
         0, 0, 1, d3,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA45() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a2,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA67() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, 0,
         0, 0, -1, 0,
         0, 1, 0, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA78(double th5) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th5), -cos(-half_pi) * sin(th5), sin(-half_pi) * sin(th5), 0,
         sin(th5), cos(-half_pi) * cos(th5), -sin(-half_pi) * cos(th5), 0,
         0, sin(-half_pi), cos(-half_pi), d5,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d InverseKinematics::calculateA89(double th6) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th6), -cos(half_pi) * sin(th6), sin(half_pi) * sin(th6), 0,
         sin(th6), cos(half_pi) * cos(th6), -sin(half_pi) * cos(th6), 0,
         0, sin(half_pi), cos(half_pi), -d6,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix3d InverseKinematics::calculateRotationMatrix(double rx, double ry, double rz) const {
    Eigen::Matrix3d Rz, Ry, Rx;
    Rz << cos(rz), -sin(rz), 0,
          sin(rz), cos(rz), 0,
          0, 0, 1;
    Ry << cos(ry), 0, sin(ry),
          0, 1, 0,
          -sin(ry), 0, cos(ry);
    Rx << 1, 0, 0,
          0, cos(rx), -sin(rx),
          0, sin(rx), cos(rx);
    return Rz * Ry * Rx;
}

bool InverseKinematics::isValidJointAngle(int joint_index, double angle) const {
    return angle >= joint_limits_min[joint_index] && angle <= joint_limits_max[joint_index];
}

// std::vector<IKSolution> InverseKinematics::calculateMultipleIKSolutions(
//     double input_x, double input_y, double input_z,
//     double input_rx, double input_ry, double input_rz) {
    
//     std::vector<IKSolution> solutions;

//     // Validate workspace boundaries
//     if (input_x < workspace_min_x || input_x > workspace_max_x ||
//         input_y < workspace_min_y || input_y > workspace_max_y ||
//         input_z < workspace_min_z || input_z > workspace_max_z) {
//         std::cout << "Target pose is outside the robot's workspace." << std::endl;
//         return solutions; // Return empty vector
//     }

//     // Log input pose
//     std::cout << "IK Input Pose:" << std::endl;
//     std::cout << "Position [mm]: [" << input_x << ", " << input_y << ", " << input_z << "]" << std::endl;
//     std::cout << "Orientation [deg]: [" << input_rx << ", " << input_ry << ", " << input_rz << "]" << std::endl;

//     // Convert orientation inputs from degrees to radians
//     double rx_rad = input_rx * D2R;
//     double ry_rad = input_ry * D2R;
//     double rz_rad = input_rz * D2R;

//     // Calculate rotation matrix
//     Eigen::Matrix3d R = calculateRotationMatrix(rx_rad, ry_rad, rz_rad);
//     Eigen::Vector3d Y06 = R.col(1);
//     Eigen::Vector3d P06(input_x, input_y, input_z);
//     Eigen::Vector3d P05 = P06 + d6 * Y06;

//     // Log intermediate results
//     std::cout << "P05: [" << P05[0] << ", " << P05[1] << ", " << P05[2] << "]" << std::endl;

//     // Try different configurations
//     std::array<int, 2> configs{-1, 1};
//     for (int shoulder_config : configs) {  // LEFTY/RIGHTY
//         try {
//             double th1 = atan2(P05[1], P05[0]) +
//                          shoulder_config * safe_acos(d4 / sqrt(pow(P05[1], 2) + pow(P05[0], 2))) +
//                          0.5 * M_PI;
//             std::cout << "th1: " << th1 * R2D << std::endl;

//             if (!isValidJointAngle(0, th1)) continue;

//             for (int wrist_config : configs) {  // FLIP/NOFLIP
//                 double th5 = wrist_config * safe_acos((sin(th1) * P06[0] - cos(th1) * P06[1] - d4) / d6);

//                 if (!isValidJointAngle(4, th5)) continue;

//                 if (!std::isnan(th5)) {
//                     std::cout << "th5: " << th5*R2D << std::endl;

//                     double th6 = atan2(-(-sin(th1) * R(0, 0) + cos(th1) * R(1, 0)) / sin(th5),
//                                        (-sin(th1) * R(0, 2) + cos(th1) * R(1, 2)) / sin(th5)) + 0.5 * M_PI;

//                     std::cout << "th6: " << th6*R2D << std::endl;
//                     if (!isValidJointAngle(5, th6)) continue;

//                     Eigen::Matrix4d A01 = calculateA01(th1);
//                     Eigen::Matrix4d A67 = calculateA67();
//                     Eigen::Matrix4d A78 = calculateA78(th5);
//                     Eigen::Matrix4d A89 = calculateA89(th6);

//                     Eigen::Matrix4d P06_homogeneous;
//                     P06_homogeneous.setIdentity();
//                     P06_homogeneous.block<3, 3>(0, 0) = R;
//                     P06_homogeneous.block<3, 1>(0, 3) = P06;

//                     Eigen::Matrix4d A17 = A01.inverse() * P06_homogeneous *
//                                           A89.inverse() * A78.inverse() * A67.inverse();

//                     Eigen::Vector3d P14 = A17.block<3, 1>(0, 3);

//                     for (int elbow_config : configs) {  // ELBOW-UP/DOWN
//                         try {
//                             double th3 = elbow_config * safe_acos(
//                                 (P14[0] * P14[0] + P14[1] * P14[1] - a1 * a1 - a2 * a2) /
//                                 (2 * a1 * a2));

//                             if (!isValidJointAngle(2, th3)) continue;

//                             if (!std::isnan(th3)) {
//                                 double th2 = atan2(P14[0], -P14[1]) -
//                                              safe_asin(a2 * sin(th3) / sqrt(P14[0] * P14[0] + P14[1] * P14[1]));

//                                 if (!isValidJointAngle(1, th2)) continue;

//                                 Eigen::Matrix4d A12 = calculateA12(th2);
//                                 Eigen::Matrix4d A23 = calculateA23();
//                                 Eigen::Matrix4d A34 = calculateA34(th3);
//                                 Eigen::Matrix4d A45 = calculateA45();

//                                 Eigen::Matrix4d A56_cal = A45.inverse() * A34.inverse() *
//                                                           A23.inverse() * A12.inverse() * A01.inverse() *
//                                                           P06_homogeneous * A89.inverse() *
//                                                           A78.inverse() * A67.inverse();

//                                 double th4 = atan2(A56_cal(1, 0), A56_cal(0, 0)) - 0.5 * M_PI;

//                                 if (!isValidJointAngle(3, th4)) continue;

//                                 // Create solution
//                                 IKSolution solution;
//                                 solution.joints = {
//                                     th1 * R2D, th2 * R2D, th3 * R2D,
//                                     th4 * R2D, th5 * R2D, th6 * R2D
//                                 };
//                                 solution.configuration = {
//                                     shoulder_config == -1 ? "LEFTY" : "RIGHTY",
//                                     elbow_config == 1 ? "UP" : "DOWN",
//                                     wrist_config == 1 ? "FLIP" : "NOFLIP"
//                                 };
//                                 solutions.push_back(solution);
//                             }
//                         } catch (...) {
//                             continue;
//                         }
//                     }
//                 }
//             }
//         } catch (...) {
//             continue;
//         }
//     }
//     return solutions;
// }

std::vector<IKSolution> InverseKinematics::calculateMultipleIKSolutions(
    double input_x, double input_y, double input_z,
    double input_rx, double input_ry, double input_rz) {
    
    std::vector<IKSolution> solutions;

    // Validate workspace boundaries using the shared workspace limits
    if (!WorkspaceLimits::isPointInWorkspace(input_x, input_y, input_z)) {
        std::cout << "Target pose is outside the robot's workspace." << std::endl;
        return solutions; // Return empty vector
    }

    // Log input pose
    std::cout << "IK Input Pose:" << std::endl;
    std::cout << "Position [mm]: [" << input_x << ", " << input_y << ", " << input_z << "]" << std::endl;
    std::cout << "Orientation [deg]: [" << input_rx << ", " << input_ry << ", " << input_rz << "]" << std::endl;

    // Convert orientation inputs from degrees to radians
    double rx_rad = input_rx * D2R;
    double ry_rad = input_ry * D2R;
    double rz_rad = input_rz * D2R;

    // Calculate rotation matrix
    Eigen::Matrix3d R = calculateRotationMatrix(rx_rad, ry_rad, rz_rad);
    Eigen::Vector3d Y06 = R.col(1);
    Eigen::Vector3d P06(input_x, input_y, input_z);
    Eigen::Vector3d P05 = P06 + d6 * Y06;

    // Log intermediate results
    std::cout << "P05: [" << P05[0] << ", " << P05[1] << ", " << P05[2] << "]" << std::endl;

    // Rest of the code remains unchanged
    
    // Try different configurations
    std::array<int, 2> configs{-1, 1};
    for (int shoulder_config : configs) {  // LEFTY/RIGHTY
        try {
            double th1 = atan2(P05[1], P05[0]) +
                         shoulder_config * safe_acos(d4 / sqrt(pow(P05[1], 2) + pow(P05[0], 2))) +
                         0.5 * M_PI;
            std::cout << "th1: " << th1 * R2D << std::endl;

            if (!isValidJointAngle(0, th1)) continue;

            for (int wrist_config : configs) {  // FLIP/NOFLIP
                double th5 = wrist_config * safe_acos((sin(th1) * P06[0] - cos(th1) * P06[1] - d4) / d6);

                if (!isValidJointAngle(4, th5)) continue;

                if (!std::isnan(th5)) {
                    std::cout << "th5: " << th5*R2D << std::endl;

                    double th6 = atan2(-(-sin(th1) * R(0, 0) + cos(th1) * R(1, 0)) / sin(th5),
                                       (-sin(th1) * R(0, 2) + cos(th1) * R(1, 2)) / sin(th5)) + 0.5 * M_PI;

                    std::cout << "th6: " << th6*R2D << std::endl;
                    if (!isValidJointAngle(5, th6)) continue;

                    Eigen::Matrix4d A01 = calculateA01(th1);
                    Eigen::Matrix4d A67 = calculateA67();
                    Eigen::Matrix4d A78 = calculateA78(th5);
                    Eigen::Matrix4d A89 = calculateA89(th6);

                    Eigen::Matrix4d P06_homogeneous;
                    P06_homogeneous.setIdentity();
                    P06_homogeneous.block<3, 3>(0, 0) = R;
                    P06_homogeneous.block<3, 1>(0, 3) = P06;

                    Eigen::Matrix4d A17 = A01.inverse() * P06_homogeneous *
                                          A89.inverse() * A78.inverse() * A67.inverse();

                    Eigen::Vector3d P14 = A17.block<3, 1>(0, 3);

                    for (int elbow_config : configs) {  // ELBOW-UP/DOWN
                        try {
                            double th3 = elbow_config * safe_acos(
                                (P14[0] * P14[0] + P14[1] * P14[1] - a1 * a1 - a2 * a2) /
                                (2 * a1 * a2));

                            if (!isValidJointAngle(2, th3)) continue;

                            if (!std::isnan(th3)) {
                                double th2 = atan2(P14[0], -P14[1]) -
                                             safe_asin(a2 * sin(th3) / sqrt(P14[0] * P14[0] + P14[1] * P14[1]));

                                if (!isValidJointAngle(1, th2)) continue;

                                Eigen::Matrix4d A12 = calculateA12(th2);
                                Eigen::Matrix4d A23 = calculateA23();
                                Eigen::Matrix4d A34 = calculateA34(th3);
                                Eigen::Matrix4d A45 = calculateA45();

                                Eigen::Matrix4d A56_cal = A45.inverse() * A34.inverse() *
                                                          A23.inverse() * A12.inverse() * A01.inverse() *
                                                          P06_homogeneous * A89.inverse() *
                                                          A78.inverse() * A67.inverse();

                                double th4 = atan2(A56_cal(1, 0), A56_cal(0, 0)) - 0.5 * M_PI;

                                if (!isValidJointAngle(3, th4)) continue;

                                // Create solution
                                IKSolution solution;
                                solution.joints = {
                                    th1 * R2D, th2 * R2D, th3 * R2D,
                                    th4 * R2D, th5 * R2D, th6 * R2D
                                };
                                solution.configuration = {
                                    shoulder_config == -1 ? "LEFTY" : "RIGHTY",
                                    elbow_config == 1 ? "UP" : "DOWN",
                                    wrist_config == 1 ? "FLIP" : "NOFLIP"
                                };
                                solutions.push_back(solution);
                            }
                        } catch (...) {
                            continue;
                        }
                    }
                }
            }
        } catch (...) {
            continue;
        }
    }
    return solutions;
}

double InverseKinematics::safe_acos(double x) const {
    return acos(std::clamp(x, -1.0, 1.0));
}

double InverseKinematics::safe_asin(double x) const {
    return asin(std::clamp(x, -1.0, 1.0));
}