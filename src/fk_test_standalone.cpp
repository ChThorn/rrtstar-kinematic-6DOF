// #include <iostream>
// #include <iomanip>
// #include "forward_kinematics.h"
// #include "inverse_kinematics.h"

// void printJoints(const std::array<double, 6>& joints) {
//     std::cout << std::fixed << std::setprecision(3);
//     std::cout << "Joints [deg]: [";
//     for (int i = 0; i < 6; ++i) {
//         if (i > 0) std::cout << ", ";
//         std::cout << joints[i];
//     }
//     std::cout << "]" << std::endl;
// }

// void printPose(const ForwardKinematics::Point& pose) {
//     std::cout << std::fixed << std::setprecision(3);
//     std::cout << "Position [mm]: ["
//               << pose[0] << ", " 
//               << pose[1] << ", " 
//               << pose[2] << "]" << std::endl;
//     std::cout << "Orientation [deg]: ["
//               << pose[3] * 180/M_PI << ", "
//               << pose[4] * 180/M_PI << ", "
//               << pose[5] * 180/M_PI << "]" << std::endl;
// }

// int main() {
//     ForwardKinematics fk;
//     InverseKinematics ik;

//     // Test Case 1: Home Position
//     std::cout << "\nTest Case 1: Home Position" << std::endl;
//     std::cout << "------------------------" << std::endl;
    
//     ForwardKinematics::Joint home_joints = {0, 0, 0, 0, 0, 0};
//     std::cout << "Input ";
//     printJoints(home_joints);
    
//     // Calculate FK
//     auto tcp_pose = fk.calculateFK(home_joints);
//     std::cout << "FK Result:" << std::endl;
//     printPose(tcp_pose);
    
//     // Calculate IK
//     auto ik_solutions = ik.calculateMultipleIKSolutions(
//         tcp_pose[0], tcp_pose[1], tcp_pose[2],
//         tcp_pose[3], tcp_pose[4], tcp_pose[5]);
    
//     std::cout << "IK Solutions Found: " << ik_solutions.size() << std::endl;
//     for (const auto& sol : ik_solutions) {
//         std::cout << "\nConfiguration: "
//                   << sol.configuration.shoulder << ", "
//                   << sol.configuration.elbow << ", "
//                   << sol.configuration.wrist << std::endl;
//         printJoints(sol.joints);
//     }

//     // Test Case 2: 90-degree rotation of first joint
//     std::cout << "\nTest Case 2: 90-degree Base Rotation" << std::endl;
//     std::cout << "--------------------------------" << std::endl;
    
//     ForwardKinematics::Joint rotated_joints = {90, 0, 0, 0, 0, 0};
//     std::cout << "Input ";
//     printJoints(rotated_joints);
    
//     // Apply FK
//     tcp_pose = fk.calculateFK(rotated_joints);
//     std::cout << "FK Result:" << std::endl;
//     printPose(tcp_pose);
    
//     // Apply IK
//     ik_solutions = ik.calculateMultipleIKSolutions(
//         tcp_pose[0], tcp_pose[1], tcp_pose[2],
//         tcp_pose[3], tcp_pose[4], tcp_pose[5]);
    
//     std::cout << "IK Solutions Found: " << ik_solutions.size() << std::endl;
//     for (const auto& sol : ik_solutions) {
//         std::cout << "\nConfiguration: "
//                   << sol.configuration.shoulder << ", "
//                   << sol.configuration.elbow << ", "
//                   << sol.configuration.wrist << std::endl;
//         printJoints(sol.joints);
//     }

//     return 0;
// }

#include <iostream>
#include <iomanip>
#include <array>
#include "forward_kinematics.h"
#include "inverse_kinematics.h"

// Function to print joint angles
void printJoints(const std::array<double, 6>& joints) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Joints [deg]: [";
    for (int i = 0; i < 6; ++i) {
        if (i > 0) std::cout << ", ";
        std::cout << joints[i];
    }
    std::cout << "]" << std::endl;
}

// Function to print pose (position in mm, orientation in degrees)
void printPose(const ForwardKinematics::Point& pose) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Position [mm]: ["
              << pose[0] << ", " 
              << pose[1] << ", " 
              << pose[2] << "]" << std::endl;
    std::cout << "Orientation [deg]: ["
              << pose[3] * 180 / M_PI << ", "
              << pose[4] * 180 / M_PI << ", "
              << pose[5] * 180 / M_PI << "]" << std::endl;
}

double evaluateSolution(const IKSolution& solution, const ForwardKinematics::Joint& current_joints) {
    double score = 0.0;

    // Criterion 1: Joint Angle Differences (Minimize total angular displacement)
    for (int i = 0; i < 6; ++i) {
        double delta = std::abs(solution.joints[i] - current_joints[i]);
        score += delta; // Add the absolute difference for each joint
    }

    // Criterion 2: Physical Constraints (Penalize solutions near joint limits)
    for (int i = 0; i < 6; ++i) {
        double lower_limit = InverseKinematics::joint_limits_min[i];
        double upper_limit = InverseKinematics::joint_limits_max[i];
        double margin = 5.0; // Allow a small margin near limits

        if (solution.joints[i] < lower_limit + margin || solution.joints[i] > upper_limit - margin) {
            score += 100.0; // Penalize solutions near joint limits
        }
    }

    // Criterion 3: Singularity Avoidance (Penalize solutions near singularities)
    if (std::abs(sin(solution.joints[4] * M_PI / 180.0)) < 0.1) { // Check for singularity in joint 5
        score += 1000.0; // Heavily penalize singular configurations
    }

    return score;
}

const IKSolution* chooseBestSolution(const std::vector<IKSolution>& solutions, const ForwardKinematics::Joint& current_joints) {
    const IKSolution* best_solution = nullptr;
    double best_score = std::numeric_limits<double>::max();

    for (const auto& sol : solutions) {
        double score = evaluateSolution(sol, current_joints);
        if (score < best_score) {
            best_score = score;
            best_solution = &sol;
        }
    }

    return best_solution;
}

int main() {
    ForwardKinematics fk;
    InverseKinematics ik;

    // Test Case 1: Home Position
    // std::cout << "\nTest Case 1: Home Position" << std::endl;
    // std::cout << "------------------------" << std::endl;
    // ForwardKinematics::Joint home_joints = {0, 0, 0, 0, 0, 0}; // All joint angles in degrees
    // std::cout << "Input Joints: ";
    // printJoints(home_joints);

    // // Calculate FK
    // auto tcp_pose = fk.calculateFK(home_joints);
    // std::cout << "FK Result:" << std::endl;
    // printPose(tcp_pose);

    // // Calculate IK
    // auto ik_solutions = ik.calculateMultipleIKSolutions(
    //     tcp_pose[0], tcp_pose[1], tcp_pose[2],
    //     tcp_pose[3] * 180 / M_PI, tcp_pose[4] * 180 / M_PI, tcp_pose[5] * 180 / M_PI); // Convert radians to degrees
    // std::cout << "IK Solutions Found: " << ik_solutions.size() << std::endl;
    // for (const auto& sol : ik_solutions) {
    //     std::cout << "\nConfiguration: "
    //               << sol.configuration.shoulder << ", "
    //               << sol.configuration.elbow << ", "
    //               << sol.configuration.wrist << std::endl;
    //     printJoints(sol.joints);

    //     // Convert IK solution back to joint angles
    //     ForwardKinematics::Joint ik_joints = {
    //         sol.joints[0], sol.joints[1], sol.joints[2],
    //         sol.joints[3], sol.joints[4], sol.joints[5]
    //     };

    //     // Calculate FK for the IK solution
    //     auto fk_pose = fk.calculateFK(ik_joints);
    //     std::cout << "FK Result for IK Solution:" << std::endl;
    //     printPose(fk_pose);
    // }

    // Test Case 1: Home Position
    std::cout << "\nTest Case 1: Home Position" << std::endl;
    ForwardKinematics::Joint home_joints = {0, 0, 0, 0, 0, 0};
    std::cout<< "Input ";
    printJoints(home_joints);

    // Calculate FK
    auto tcp_pose = fk.calculateFK(home_joints);
    std::cout << "FK Result: " << std::endl;
    printPose(tcp_pose);

    // Calculate IK
    auto ik_solutions = ik.calculateMultipleIKSolutions(
        tcp_pose[0], tcp_pose[1], tcp_pose[2],
        tcp_pose[3], tcp_pose[4], tcp_pose[5]
    );

    std::cout << "IK Solutions Found: "<<ik_solutions.size() << std::endl;

    // Choose the best solution
    const IKSolution* best_solution = chooseBestSolution(ik_solutions, home_joints);
    if(best_solution)
    {
        std::cout << "\nBest Solution Selected: " << std::endl;
        std::cout << "Configuration: "
                  << best_solution->configuration.shoulder << ", "
                  << best_solution->configuration.elbow << ", "
                  << best_solution->configuration.wrist << std::endl;
        printJoints(best_solution->joints);
    }
    else
    {
        std::cout << "No valid IK solution found." << std::endl;
    }

    // Test Case 2: 90-degree Base Rotation
    std::cout << "\nTest Case 2: 90-degree Base Rotation" << std::endl;
    std::cout << "--------------------------------" << std::endl;
    ForwardKinematics::Joint rotated_joints = {90, 0, 0, 0, 0, 0}; // First joint rotated 90 degrees
    std::cout << "Input Joints: ";
    printJoints(rotated_joints);

    // Apply FK
    tcp_pose = fk.calculateFK(rotated_joints);
    std::cout << "FK Result:" << std::endl;
    printPose(tcp_pose);

    // Apply IK
    ik_solutions = ik.calculateMultipleIKSolutions(
        tcp_pose[0], tcp_pose[1], tcp_pose[2],
        tcp_pose[3] * 180 / M_PI, tcp_pose[4] * 180 / M_PI, tcp_pose[5] * 180 / M_PI); // Convert radians to degrees
    std::cout << "IK Solutions Found: " << ik_solutions.size() << std::endl;
    for (const auto& sol : ik_solutions) {
        std::cout << "\nConfiguration: "
                  << sol.configuration.shoulder << ", "
                  << sol.configuration.elbow << ", "
                  << sol.configuration.wrist << std::endl;
        printJoints(sol.joints);

        // Convert IK solution back to joint angles
        // ForwardKinematics::Joint ik_joints = {
        //     sol.joints[0], sol.joints[1], sol.joints[2],
        //     sol.joints[3], sol.joints[4], sol.joints[5]
        // };

        // // Calculate FK for the IK solution
        // auto fk_pose = fk.calculateFK(ik_joints);
        // std::cout << "FK Result for IK Solution:" << std::endl;
        // printPose(fk_pose);
    }

    // Test Case 3: Singularity - Fully Extended Arm
    std::cout << "\nTest Case 3: Singularity - Fully Extended Arm" << std::endl;
    ForwardKinematics::Joint singularity_joints = {0, 90, 0, 0, 0, 0}; // Fully extended
    std::cout << "Input Joints: ";
    printJoints(singularity_joints);

    auto singularity_pose = fk.calculateFK(singularity_joints);
    std::cout << "FK Result:" << std::endl;
    printPose(singularity_pose);

    auto singularity_ik_solutions = ik.calculateMultipleIKSolutions(
        singularity_pose[0], singularity_pose[1], singularity_pose[2],
        singularity_pose[3] * 180 / M_PI, singularity_pose[4] * 180 / M_PI, singularity_pose[5] * 180 / M_PI);
    std::cout << "IK Solutions Found: " << singularity_ik_solutions.size() << std::endl;

    // Test Case 4: Extreme Joint Angles
    std::cout << "\nTest Case 4: Extreme Joint Angles" << std::endl;
    ForwardKinematics::Joint extreme_joints = {-170, 120, -225, 180, -97, 180}; // Near joint limits
    std::cout << "Input Joints: ";
    printJoints(extreme_joints);

    auto extreme_pose = fk.calculateFK(extreme_joints);
    std::cout << "FK Result:" << std::endl;
    printPose(extreme_pose);

    auto extreme_ik_solutions = ik.calculateMultipleIKSolutions(
        extreme_pose[0], extreme_pose[1], extreme_pose[2],
        extreme_pose[3] * 180 / M_PI, extreme_pose[4] * 180 / M_PI, extreme_pose[5] * 180 / M_PI);
    std::cout << "IK Solutions Found: " << extreme_ik_solutions.size() << std::endl;

    // Test Case 5: Unreachable Pose
    std::cout << "\nTest Case 5: Unreachable Pose" << std::endl;
    double unreachable_x = 2000, unreachable_y = 2000, unreachable_z = 2000; // Outside workspace
    double unreachable_rx = 0, unreachable_ry = 0, unreachable_rz = 0;

    auto unreachable_ik_solutions = ik.calculateMultipleIKSolutions(
        unreachable_x, unreachable_y, unreachable_z,
        unreachable_rx, unreachable_ry, unreachable_rz);
    std::cout << "IK Solutions Found: " << unreachable_ik_solutions.size() << std::endl;

    return 0;
}