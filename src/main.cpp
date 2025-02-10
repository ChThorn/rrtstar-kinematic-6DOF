#include "rrtstar.h"
#include <iostream>

// int main() {
//     // Define start and goal configurations in joint space
//     std::array<double, 6> start_config = {0, 0, 0, 0, 0, 0};  // Home position
//     std::array<double, 6> goal_config = {M_PI / 32, M_PI / 48, -M_PI / 48, 0, M_PI / 48, 0};

//     try {
//         RRTStar rrtstar(
//             start_config,
//             goal_config,
//             400,    // Larger workspace width
//             400,    // Larger workspace height
//             400,    // Larger workspace depth
//             0.05,   // Step size
//             0.25,   // Neighbor radius
//             0.1,    // Safety margin
//             10000,  // Max iterations
//             -100,      // Center workspace (x)
//             -100,      // Center workspace (y)
//             -100       // Center workspace (z)
//         );

//         // Find path
//         std::cout << "Planning path..." << std::endl;
//         auto path = rrtstar.findPath();
//         if (path.empty()) {
//             std::cout << "No path found!" << std::endl;
//             return 1;
//         }
//         std::cout << "Path found with " << path.size() << " nodes!" << std::endl;

//         // Print path
//         std::cout << "\nPath (joint configurations):" << std::endl;
//         for (const auto* node : path) {
//             std::cout << "Joint values: ";
//             for (size_t i = 0; i < 6; ++i) {
//                 std::cout << node->q[i] << " ";
//             }
//             auto pos = node->end_effector_position();
//             std::cout << "| EE position: " << pos[0] << " " << pos[1] << " " << pos[2] << std::endl;
//         }

//         // Visualize path
//         rrtstar.visualizePath(path);
//         return 0;
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return 1;
//     }
// }

// #include "rrtstar.h"
// #include <iostream>
// #include <iomanip>

// // Helper function to convert degrees to radians
// double deg2rad(double deg) {
//     return deg * M_PI / 180.0;
// }

// // Helper function to normalize angle to [-π, π]
// double normalizeAngle(double angle) {
//     angle = fmod(angle + M_PI, 2 * M_PI);
//     if (angle < 0) angle += 2 * M_PI;
//     return angle - M_PI;
// }

// int main() {
//     try {
//         // Define start configuration in joint space
//         std::array<double, 6> start_config = {0, 0, 0, 0, 0, 0};
//         auto start_pos = RobotKinematics::computeFK(start_config);
//         std::cout << "Start Position: " 
//                   << start_pos[0] << ", " 
//                   << start_pos[1] << ", " 
//                   << start_pos[2] << std::endl;

        

//         // Define goal in Cartesian space
//         // double goal_x = -156.76, goal_y = -155.15, goal_z = 814.96;
//         // double goal_rx = -43.47, goal_ry = 80.56, goal_rz = -60.88;
//         double goal_x = 300.0, goal_y = 300.0, goal_z = 500.0;
//         double goal_rx = 0.0, goal_ry = 0.0, goal_rz = 0.0;
        

//         // Create planner with optimized parameters
//         RRTStar rrtstar_planner(
//             start_config, 
//             {}, 
//             2000,   // Workspace width
//             2000,   // Workspace height
//             1500,   // Workspace depth
//             0.5,    //0.1 Increased step size for faster coverage
//             5.0,    //0.5 Increased neighbor radius for better optimization
//             0.1,    //0.1 Safety margin
//             5000,   // Reduced max iterations
//             -1000,   // Min x
//             -1000,   // Min y
//             0       // Min z
//         );

//         std::cout << "Converting Cartesian goal to joint space..." << std::endl;
//         // auto goal_config = rrtstar_planner.cartesianToJointSpace(
//         //     goal_x, goal_y, goal_z, goal_rx, goal_ry, goal_rz);
//         auto goal_config = rrtstar_planner.cartesianToJointSpace(goal_x, goal_y, goal_z, goal_rx, goal_ry, goal_rz);

//         // Initialize RRT* with optimized parameters
//         RRTStar rrtstar(
//             start_config,
//             goal_config,
//             2000, 2000, 1500,
//             0.5,    // Increased step size
//             5.0,    // Increased neighbor radius
//             0.1,    // Safety margin
//             5000,   // Reduced max iterations
//             -1000, -1000, 0
//         );

//         // Find path
//         std::cout << "Planning path..." << std::endl;
//         auto path = rrtstar.findPath();
//         if (path.empty()) {
//             std::cout << "No path found!" << std::endl;
//             return 1;
//         }

//         std::cout << "\nPath found with " << path.size() << " nodes!" << std::endl;
        
//         // Print path details
//         std::cout << "\nPath waypoints:" << std::endl;
//         for (size_t i = 0; i < path.size(); ++i) {
//             auto pos = path[i]->end_effector_position();
//             std::cout << "Node " << i << ":" << std::endl;
//             std::cout << "  Position: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
//         }

//         // Visualize and save path
//         rrtstar.visualizePath(path);

//         // Save and display results
//         RRTStar::savePlanningResults(path);
        
//         return 0;
//     } catch (const std::exception& e) {
//         std::cerr << "Error: " << e.what() << std::endl;
//         return 1;
//     }
// }


#include "rrtstar.h"
#include <iostream>
#include <iomanip>

// Helper function to convert degrees to radians
double deg2rad(double deg) {
    return deg * M_PI / 180.0;
}

// Helper function to normalize angle to [-π, π]
double normalizeAngle(double angle) {
    angle = fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    return angle - M_PI;
}

int main() {
    try {
        // Define start configuration in joint space
        std::array<double, 6> start_config = {0, 0, 0, 0, 0, 0};
        auto start_pos = RobotKinematics::computeFK(start_config);
        std::cout << "Start Position: " 
                  << start_pos[0] << ", " 
                  << start_pos[1] << ", " 
                  << start_pos[2] << std::endl;

        // Define goal in Cartesian space
        double goal_x = 300.0, goal_y = 300.0, goal_z = 500.0;
        double goal_rx = 0.0, goal_ry = 0.0, goal_rz = 0.0;

        // Test IK solutions for the goal position
        std::cout << "\n=== Testing IK solutions for goal position ===" << std::endl;
        std::cout << "Goal: X=" << goal_x << ", Y=" << goal_y << ", Z=" << goal_z 
                  << ", RX=" << goal_rx << ", RY=" << goal_ry << ", RZ=" << goal_rz << std::endl;

        InverseKinematics ik_solver;
        auto ik_solutions = ik_solver.calculateMultipleIKSolutions(
            goal_x, goal_y, goal_z,
            goal_rx, goal_ry, goal_rz
        );

        std::cout << "\nFound " << ik_solutions.size() << " IK solutions:" << std::endl;
        for (size_t i = 0; i < ik_solutions.size(); ++i) {
            const auto& sol = ik_solutions[i];
            std::cout << "\nSolution " << (i + 1) << ":" << std::endl;
            std::cout << "Configuration: " 
                      << "Shoulder=" << sol.configuration.shoulder << ", "
                      << "Elbow=" << sol.configuration.elbow << ", "
                      << "Wrist=" << sol.configuration.wrist << std::endl;
            std::cout << "Joint angles (degrees): ";
            for (const auto& angle : sol.joints) {
                std::cout << std::fixed << std::setprecision(3) << angle << " ";
            }
            std::cout << std::endl;
        }

        if (ik_solutions.empty()) {
            std::cout << "No IK solutions found for the goal position!" << std::endl;
            return 1;
        }

        // Create planner with optimized parameters
        RRTStar rrtstar_planner(
            start_config, 
            {}, 
            2000,   // Workspace width
            2000,   // Workspace height
            1500,   // Workspace depth
            0.5,    // Step size
            5.0,    // Neighbor radius
            0.1,    // Safety margin
            5000,   // Max iterations
            -1000,  // Min x
            -1000,  // Min y
            0       // Min z
        );

        std::cout << "\n=== Converting Cartesian goal to joint space ===" << std::endl;
        auto goal_config = rrtstar_planner.cartesianToJointSpace(
            goal_x, goal_y, goal_z, goal_rx, goal_ry, goal_rz);

        // Print chosen goal configuration
        std::cout << "Chosen goal configuration (radians): ";
        for (const auto& angle : goal_config) {
            std::cout << std::fixed << std::setprecision(3) << angle << " ";
        }
        std::cout << std::endl;

        // Initialize RRT* with the configurations
        RRTStar rrtstar(
            start_config,
            goal_config,
            2000, 2000, 1500,
            0.5,    // Step size
            5.0,    // Neighbor radius
            0.1,    // Safety margin
            5000,   // Max iterations
            -1000, -1000, 0
        );

        // Find path
        std::cout << "\n=== Planning path ===" << std::endl;
        auto path = rrtstar.findPath();
        if (path.empty()) {
            std::cout << "No path found!" << std::endl;
            return 1;
        }

        std::cout << "\nPath found with " << path.size() << " nodes!" << std::endl;
        
        // Print path details
        std::cout << "\nPath waypoints:" << std::endl;
        for (size_t i = 0; i < path.size(); ++i) {
            auto pos = path[i]->end_effector_position();
            std::cout << "Node " << i << ":" << std::endl;
            std::cout << "  Position: (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")" << std::endl;
            std::cout << "  Joint angles: ";
            for (const auto& angle : path[i]->q) {
                std::cout << std::fixed << std::setprecision(3) << angle << " ";
            }
            std::cout << std::endl;
        }

        // Visualize and save path
        rrtstar.visualizePath(path);
        RRTStar::savePlanningResults(path);
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}