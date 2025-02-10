#include "ivk.h"
#include <iostream>

Eigen::Matrix4d Ivk::calculateA01(double th1) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th1), -cos(-half_pi) * sin(th1), sin(-half_pi) * sin(th1), 0,
         sin(th1), cos(-half_pi) * cos(th1), -sin(-half_pi) * cos(th1), 0,
         0, sin(-half_pi), cos(-half_pi), d1,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA12(double th2) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th2 - half_pi), -sin(th2 - half_pi), 0, 0,
         sin(th2 - half_pi), cos(th2 - half_pi), 0, 0,
         0, 0, 1, -d2,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA23() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a1,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA34(double th3) const {
    Eigen::Matrix4d A;
    A << cos(th3), -sin(th3), 0, 0,
         sin(th3), cos(th3), 0, 0,
         0, 0, 1, d3,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA45() const {
    Eigen::Matrix4d A;
    A << 1, 0, 0, a2,
         0, 1, 0, 0,
         0, 0, 1, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA67() const {
    Eigen::Matrix4d A;
    // double half_pi = M_PI / 2.0;
    A << 1, 0, 0, 0,
         0, 0, -1, 0,
         0, 1, 0, 0,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA78(double th5) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th5), -cos(-half_pi) * sin(th5), sin(-half_pi) * sin(th5), 0,
         sin(th5), cos(-half_pi) * cos(th5), -sin(-half_pi) * cos(th5), 0,
         0, sin(-half_pi), cos(-half_pi), d5,
         0, 0, 0, 1;
    return A;
}

Eigen::Matrix4d Ivk::calculateA89(double th6) const {
    Eigen::Matrix4d A;
    double half_pi = M_PI / 2.0;
    A << cos(th6), -cos(half_pi) * sin(th6), sin(half_pi) * sin(th6), 0,
         sin(th6), cos(half_pi) * cos(th6), -sin(half_pi) * cos(th6), 0,
         0, sin(half_pi), cos(half_pi), -d6,
         0, 0, 0, 1;
    return A;
}



Eigen::Matrix3d Ivk::calculateRotationMatrix(double rx, double ry, double rz) const {
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

std::vector<IKSolution> Ivk::calculateMultipleIKSolutions(
    double input_x, double input_y, double input_z,
    double input_rx, double input_ry, double input_rz) {
    
    std::vector<IKSolution> solutions;

    // Convert angles to radians
    double rx = input_rx * D2R;
    double ry = input_ry * D2R;
    double rz = input_rz * D2R;

    // Calculate rotation matrix
    Eigen::Matrix3d R = calculateRotationMatrix(rx, ry, rz);
    Eigen::Vector3d Y06 = R.col(1);
    Eigen::Vector3d P06(input_x, input_y, input_z);
    Eigen::Vector3d P05 = P06 + d6 * Y06;

    // Try different configurations
    std::array<int, 2> configs{-1, 1};
    
    for (int shoulder_config : configs) {  // LEFTY/RIGHTY
        try {
            double th1 = atan2(P05[1], P05[0]) + 
                        shoulder_config * acos(d4 / sqrt(pow(P05[1], 2) + pow(P05[0], 2))) + 
                        0.5 * M_PI;

            for (int wrist_config : configs) {  // FLIP/NOFLIP
                double th5 = wrist_config * acos((sin(th1) * P06[0] - cos(th1) * P06[1] - d4) / d6);
                
                if (!std::isnan(th5)) {
                    double th6 = atan2(-(-sin(th1) * R(0, 0) + cos(th1) * R(1, 0)) / sin(th5),
                                     (-sin(th1) * R(0, 2) + cos(th1) * R(1, 2)) / sin(th5)) + 0.5 * M_PI;

                    // Calculate intermediate transformations
                    Eigen::Matrix4d A01 = calculateA01(th1);
                    Eigen::Matrix4d A67 = calculateA67();
                    Eigen::Matrix4d A78 = calculateA78(th5);
                    Eigen::Matrix4d A89 = calculateA89(th6);

                    Eigen::Matrix4d P06_homogeneous;
                    P06_homogeneous.setIdentity();
                    P06_homogeneous.block<3,3>(0,0) = R;
                    P06_homogeneous.block<3,1>(0,3) = P06;

                    Eigen::Matrix4d A17 = A01.inverse() * P06_homogeneous * 
                                        A89.inverse() * A78.inverse() * A67.inverse();
                    
                    Eigen::Vector3d P14 = A17.block<3,1>(0,3);

                    for (int elbow_config : configs) {  // ELBOW-UP/DOWN
                        try {
                            double th3 = elbow_config * acos(
                                (P14[0] * P14[0] + P14[1] * P14[1] - a1 * a1 - a2 * a2) / 
                                (2 * a1 * a2));

                            if (!std::isnan(th3)) {
                                double th2 = atan2(P14[0], -P14[1]) - 
                                           asin(a2 * sin(th3) / sqrt(P14[0] * P14[0] + P14[1] * P14[1]));

                                // Calculate th4
                                Eigen::Matrix4d A12 = calculateA12(th2);
                                Eigen::Matrix4d A23 = calculateA23();
                                Eigen::Matrix4d A34 = calculateA34(th3);
                                Eigen::Matrix4d A45 = calculateA45();

                                Eigen::Matrix4d A56_cal = A45.inverse() * A34.inverse() * 
                                                        A23.inverse() * A12.inverse() * A01.inverse() * 
                                                        P06_homogeneous * A89.inverse() * 
                                                        A78.inverse() * A67.inverse();

                                double th4 = atan2(A56_cal(1,0), A56_cal(0,0)) - 0.5 * M_PI;

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

// Example main function
int main() {
    Ivk solver;
    
    // Example input values
    double input_x = 300.0;
    double input_y = 300.0;
    double input_z = 500.0;
    double input_rx = 0.0;
    double input_ry = 0.0;
    double input_rz = 0.0;
    
    auto solutions = solver.calculateMultipleIKSolutions(
        input_x, input_y, input_z, input_rx, input_ry, input_rz);
    
    std::cout << "\nFound " << solutions.size() << " valid IK solutions:" << std::endl;
    
    for (size_t i = 0; i < solutions.size(); ++i) {
        const auto& sol = solutions[i];
        std::cout << "\nSolution " << (i + 1) << ":" << std::endl;
        std::cout << "Configuration: " 
                  << "Shoulder=" << sol.configuration.shoulder << ", "
                  << "Elbow=" << sol.configuration.elbow << ", "
                  << "Wrist=" << sol.configuration.wrist << std::endl;
        
        std::cout << "Joint angles (degrees):" << std::endl;
        for (size_t j = 0; j < 6; ++j) {
            std::cout << "θ" << (j + 1) << ": " << sol.joints[j] << std::endl;
        }
    }
    
    return 0;
}