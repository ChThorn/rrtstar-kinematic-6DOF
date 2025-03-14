#include "rrtstar_main.h"
// #include "robot_kinematics.h"
#include <gtest/gtest.h>
// #include "path_export.h"
// #include "path_return.h"

const std::vector<std::array<double, 6>> reference_path = {
    {0, 0, 0, 0, 0, 0},
    {1, 1, 1, 0, 0, 0},
    {2, 2, 2, 0, 0, 0},
    {10, 10, 10, 0, 0, 0}
};

// Helper to print robot position for debugging
void printPosition(const std::array<double, 6>& q) {
    auto T = RobotKinematics::computeFK(q);
    Eigen::Vector3d pos = T.translation();
    std::cout << "Position for config " << q[0] << "," << q[1] << "," << q[2] << ","
              << q[3] << "," << q[4] << "," << q[5] << " is: ("
              << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
}

std::array<double, 6> getValidJointConfig(bool start_config = true) {
    // These should be valid joint angles for your robot
    if (start_config) {
        return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Home position
    } else {
        // A nearby but different valid configuration
        return {0.1, 0.3, 0.0, 0.0, 0.1, 0.0};
    }
}

TEST(RRTStarTests, IsStateValid) {
    RRTStarModified::updateObstacles({});
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    printPosition(valid_start);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 10, 50, 10, 1000, -500, -500, -500
    );
    
    // Test configuration validity through collision detector
    // These calls are now delegated to the CollisionDetection component
    CollisionDetection collision(10.0); // Create with same safety margin
    
    // Test valid configuration
    EXPECT_TRUE(collision.isStateValid(valid_start));
    
    // Test invalid configuration (out of joint limits)
    std::array<double, 6> invalid_q = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    EXPECT_FALSE(collision.isStateValid(invalid_q));
}

TEST(RRTStarTests, LineAABBIntersection) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 10, 50, 10, 1000, -500, -500, -500
    );
    
    // Create a collision detection object directly for testing
    CollisionDetection collision(10.0);
    
    std::array<double, 3> start = {0, 0, 0};
    std::array<double, 3> end = {5, 5, 5};
    std::array<double, 3> box_min = {1, 1, 1};
    std::array<double, 3> box_max = {4, 4, 4};

    EXPECT_TRUE(collision.lineAABBIntersection(start, end, box_min, box_max));
}

TEST(RRTStarTests, AdaptiveGoalBias) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    // Create a TreeManager directly for testing the randomization
    TreeManager tree_manager(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 9.99, 50, -500, -500, -500
    );
    
    int goal_samples = 0;
    for (int i = 0; i < 1000; ++i) {
        auto random_node = tree_manager.getRandomNode(i, 1000); // Pass iteration and max iterations
        if (random_node->q == tree_manager.getGoalConfig()) {
            goal_samples++;
        }
    }
    EXPECT_GT(goal_samples, 100); // Expect at least 10% goal samples
}

TEST(RRTStarTests, DynamicInterpolationSteps) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 9.99, 50, 10, 1000, -500, -500, -500
    );
    
    auto node1 = std::make_shared<Node>(valid_start);
    auto node2 = std::make_shared<Node>(valid_goal);

    // For testing isCollisionFree, create a PathOptimization object
    CollisionDetection collision(10.0);
    PathOptimization path_optimizer(collision, 9.99);
    
    // Get collision-free status and number of steps
    auto [collision_free, steps] = path_optimizer.isCollisionFree(node1, node2);
    EXPECT_GE(steps, 10); // Ensure at least 10 steps for large distances
}

TEST(RRTStarTests, PathSmoothing) {
    // Use valid joint configurations for your robot
    std::array<double, 6> start_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<double, 6> goal_config = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0};
    
    // Check these are valid configurations
    printPosition(start_config);
    printPosition(goal_config);
    
    RRTStarModified planner(
        start_config,
        goal_config,
        1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
    );
    
    planner.setVisualizationEnabled(false);
    
    // Create a simple path manually
    std::vector<std::shared_ptr<Node>> path;
    
    // Create a simple direct path from start to goal
    auto start_node = std::make_shared<Node>(start_config);
    
    // Create an intermediate configuration
    std::array<double, 6> mid_config = start_config;
    for (int i = 0; i < 6; i++) {
        mid_config[i] = start_config[i] + 0.5 * (goal_config[i] - start_config[i]);
    }
    auto mid_node = std::make_shared<Node>(mid_config);
    
    auto goal_node = std::make_shared<Node>(goal_config);
    
    // Set up parent relationships
    mid_node->parent = start_node;
    goal_node->parent = mid_node;
    
    // Add to path
    path.push_back(start_node);
    path.push_back(mid_node);
    path.push_back(goal_node);
    
    std::cout << "Created manual path with " << path.size() << " nodes" << std::endl;
    
    // Verify basic properties of the path
    ASSERT_GT(path.size(), 0) << "Path creation failed";
    
    // Direct access to path optimizer for testing
    CollisionDetection collision(0.1);
    PathOptimization path_optimizer(collision, 0.5);
    
    // Apply optimization
    path_optimizer.optimizePath(path);
    std::cout << "Optimized path size: " << path.size() << std::endl;
    
    // Print out the optimized path for debugging
    std::cout << "Optimized path nodes:" << std::endl;
    for (size_t i = 0; i < path.size(); ++i) {
        std::cout << "Node " << i << ": [";
        for (int j = 0; j < 6; ++j) {
            std::cout << path[i]->q[j];
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    // Verify velocity constraints on the path
    // Use constants directly since they're now public
    const double max_step = PathOptimization::MAX_JOINT_VEL / PathOptimization::CONTROL_RATE * 1.1;
    
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = TreeManager::distance(path[i-1], path[i]);
        std::cout << "Step " << i << " distance: " << dist << std::endl;
        EXPECT_LE(dist, max_step)
            << "Step " << i << " exceeds velocity constraint";
    }
    
    // Calculate path metrics
    double total_path_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        total_path_length += TreeManager::distance(path[i-1], path[i]);
    }
    std::cout << "Total path length: " << total_path_length << std::endl;
    
    // Ensure path still connects start to goal
    EXPECT_NEAR(path.front()->q[0], start_config[0], 1e-6);
    EXPECT_NEAR(path.front()->q[1], start_config[1], 1e-6);
    EXPECT_NEAR(path.back()->q[0], goal_config[0], 1e-6);
    EXPECT_NEAR(path.back()->q[1], goal_config[1], 1e-6);
}

TEST(RRTStarTests, RegressionTest) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
    );
    
    planner.setVisualizationEnabled(false);
    
    // CREATE MANUAL PATH INSTEAD OF USING PLANNER
    std::vector<std::shared_ptr<Node>> path;
    
    // Create several nodes for a more realistic path
    std::vector<std::array<double, 6>> waypoints = {
        valid_start,
        {0.03, 0.03, 0.0, 0, 0, 0},
        {0.06, 0.06, 0.0, 0, 0, 0},
        valid_goal
    };
    
    // Create nodes and build path
    for (const auto& waypoint : waypoints) {
        path.push_back(std::make_shared<Node>(waypoint));
    }
    
    // Set up parent relationships
    for (size_t i = 1; i < path.size(); ++i) {
        path[i]->parent = path[i-1];
    }
    
    std::cout << "Original path size: " << path.size() << std::endl;
    
    // Direct access to path optimizer for testing
    CollisionDetection collision(0.1);
    PathOptimization path_optimizer(collision, 0.5);
    
    // Apply optimization to get smaller steps
    path_optimizer.optimizePath(path);
    std::cout << "Optimized path size: " << path.size() << std::endl;
    
    // Verify path properties
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = TreeManager::distance(path[i-1], path[i]);
        std::cout << "Step " << i << " distance: " << dist << std::endl;
        EXPECT_LE(dist, 0.6) << "Step " << i << " exceeds maximum step size";
    }
}

// TEST(RRTStarTests, ExportPathForRobot) {
//     // Create a simple path similar to the PathSmoothing test
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     RRTStarModified planner(
//         valid_start,    // Valid start config
//         valid_goal,     // Valid goal config
//         1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
//     );
    
//     // Create a manual path
//     std::vector<std::shared_ptr<Node>> path;
//     auto start_node = std::make_shared<Node>(valid_start);
    
//     // Create intermediate configurations
//     std::array<double, 6> mid_config1 = valid_start;
//     std::array<double, 6> mid_config2 = valid_start;
//     for (int i = 0; i < 6; i++) {
//         double diff = valid_goal[i] - valid_start[i];
//         mid_config1[i] = valid_start[i] + 0.33 * diff;
//         mid_config2[i] = valid_start[i] + 0.66 * diff;
//     }
    
//     auto mid_node1 = std::make_shared<Node>(mid_config1);
//     auto mid_node2 = std::make_shared<Node>(mid_config2);
//     auto goal_node = std::make_shared<Node>(valid_goal);
    
//     // Set up parent relationships
//     mid_node1->parent = start_node;
//     mid_node2->parent = mid_node1;
//     goal_node->parent = mid_node2;
    
//     // Add to path
//     path.push_back(start_node);
//     path.push_back(mid_node1);
//     path.push_back(mid_node2);
//     path.push_back(goal_node);
    
//     // Export path for robot (using the new PathOptimization component directly)
//     CollisionDetection collision(0.1);
//     PathOptimization path_optimizer(collision, 0.5);
//     std::vector<std::array<double, 8>> robot_commands;
//     path_optimizer.exportPathForRobot(path, robot_commands);
    
//     // Print the robot commands for debugging
//     std::cout << "Robot commands for path:" << std::endl;
//     for (size_t i = 0; i < robot_commands.size(); i++) {
//         std::cout << "Command " << i << ": [";
//         for (int j = 0; j < 8; j++) {
//             std::cout << robot_commands[i][j];
//             if (j < 7) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     // Verify the number of commands matches the path size
//     ASSERT_EQ(robot_commands.size(), path.size());
    
//     // Verify joint values match the path
//     for (size_t i = 0; i < path.size(); i++) {
//         for (int j = 0; j < 6; j++) {
//             EXPECT_NEAR(robot_commands[i][j], path[i]->q[j], 1e-6);
//         }
//     }
    
//     // Verify speed and acceleration are in valid range
//     for (const auto& cmd : robot_commands) {
//         EXPECT_GE(cmd[6], 0.1);  // Speed >= 10%
//         EXPECT_LE(cmd[6], 1.0);  // Speed <= 100%
//         EXPECT_GE(cmd[7], 0.1);  // Acc >= 10%
//         EXPECT_LE(cmd[7], 1.0);  // Acc <= 100%
//     }
    
//     // Check first command has default values
//     EXPECT_NEAR(robot_commands[0][6], 0.5, 1e-6);  // Default speed
//     EXPECT_NEAR(robot_commands[0][7], 0.3, 1e-6);  // Default acc
// }

TEST(RRTStarTests, ExportPathForRobot) {
    // Create a simple path similar to the PathSmoothing test
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    // Use workspace limits from the centralized definition
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        WorkspaceLimits::WIDTH,
        WorkspaceLimits::HEIGHT,
        WorkspaceLimits::DEPTH,
        0.5, 2.0, 0.1, 2000, 
        WorkspaceLimits::MIN_X,
        WorkspaceLimits::MIN_Y,
        WorkspaceLimits::MIN_Z
    );
    
    // Create a manual path
    std::vector<std::shared_ptr<Node>> path;
    auto start_node = std::make_shared<Node>(valid_start);
    
    // Create intermediate configurations
    std::array<double, 6> mid_config1 = valid_start;
    std::array<double, 6> mid_config2 = valid_start;
    for (int i = 0; i < 6; i++) {
        double diff = valid_goal[i] - valid_start[i];
        mid_config1[i] = valid_start[i] + 0.33 * diff;
        mid_config2[i] = valid_start[i] + 0.66 * diff;
    }
    
    auto mid_node1 = std::make_shared<Node>(mid_config1);
    auto mid_node2 = std::make_shared<Node>(mid_config2);
    auto goal_node = std::make_shared<Node>(valid_goal);
    
    // Set up parent relationships
    mid_node1->parent = start_node;
    mid_node2->parent = mid_node1;
    goal_node->parent = mid_node2;
    
    // Add to path
    path.push_back(start_node);
    path.push_back(mid_node1);
    path.push_back(mid_node2);
    path.push_back(goal_node);
    
    // Export path for robot (using the new PathOptimization component directly)
    CollisionDetection collision(0.1);
    PathOptimization path_optimizer(collision, 0.5);
    std::vector<std::array<double, 8>> robot_commands;
    // path_optimizer.exportPathForRobot(path, robot_commands);
    exportPathForRobot(path, robot_commands);
    
    // Print the robot commands for debugging
    std::cout << "Robot commands for path:" << std::endl;
    for (size_t i = 0; i < robot_commands.size(); i++) {
        std::cout << "Command " << i << ": [";
        for (int j = 0; j < 8; j++) {
            std::cout << robot_commands[i][j];
            if (j < 7) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    // Verify the number of commands matches the path size
    ASSERT_EQ(robot_commands.size(), path.size());
    
    // Verify joint values match the path
    for (size_t i = 0; i < path.size(); i++) {
        for (int j = 0; j < 6; j++) {
            EXPECT_NEAR(robot_commands[i][j], path[i]->q[j], 1e-6);
        }
    }
    
    // Verify speed and acceleration are in valid range
    for (const auto& cmd : robot_commands) {
        EXPECT_GE(cmd[6], 0.1);  // Speed >= 10%
        EXPECT_LE(cmd[6], 1.0);  // Speed <= 100%
        EXPECT_GE(cmd[7], 0.1);  // Acc >= 10%
        EXPECT_LE(cmd[7], 1.0);  // Acc <= 100%
    }
    
    // Check first command has default values
    EXPECT_NEAR(robot_commands[0][6], 0.5, 1e-6);  // Default speed
    EXPECT_NEAR(robot_commands[0][7], 0.3, 1e-6);  // Default acc
}

// TEST(RRTStarTests, ExportEmptyPath) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     // Export empty path using PathOptimization directly
//     CollisionDetection collision(0.1);
//     PathOptimization path_optimizer(collision, 0.5);
    
//     // Create an empty path
//     std::vector<std::shared_ptr<Node>> empty_path;
    
//     // Export empty path
//     std::vector<std::array<double, 8>> robot_commands;
//     // path_optimizer.exportPathForRobot(empty_path, robot_commands);
//     exportPathForRobot(empty_path, robot_commands);
    
//     // Verify result is also empty
//     EXPECT_TRUE(robot_commands.empty());
// }

TEST(RRTStarTests, ExportEmptyPath) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    // Create a planner with the valid configurations
    RRTStarModified planner(
        valid_start,
        valid_goal,
        WorkspaceLimits::WIDTH,
        WorkspaceLimits::HEIGHT,
        WorkspaceLimits::DEPTH,
        0.5, 2.0, 0.1, 2000,
        WorkspaceLimits::MIN_X,
        WorkspaceLimits::MIN_Y,
        WorkspaceLimits::MIN_Z
    );
    
    // Create an empty path
    std::vector<std::shared_ptr<Node>> empty_path;
    
    // Test both the planner's method and the standalone function
    std::vector<std::array<double, 8>> robot_commands;
    
    // Use the planner to export the empty path
    planner.exportPathForRobot(empty_path, robot_commands);
    EXPECT_TRUE(robot_commands.empty());
    
    // Also test the standalone function for consistency
    exportPathForRobot(empty_path, robot_commands);
    EXPECT_TRUE(robot_commands.empty());
}

// TEST(RRTStarTests, SimpleReturnPathCreation) {
//     auto home_config = getValidJointConfig(true);
//     auto goal_config = getValidJointConfig(false);
    
//     // Create a manual forward path
//     std::vector<std::shared_ptr<Node>> forward_path;
    
//     // Add waypoints from home to goal
//     std::vector<std::array<double, 6>> waypoints = {
//         home_config,                          // A
//         {0.033, 0.033, 0.0, 0, 0.033, 0},     // C
//         {0.066, 0.066, 0.0, 0, 0.066, 0},     // D
//         goal_config                           // G
//     };
    
//     // Create nodes and build forward path
//     for (size_t i = 0; i < waypoints.size(); i++) {
//         auto node = std::make_shared<Node>(waypoints[i]);
//         if (i > 0) {
//             node->parent = forward_path.back();
//         }
//         forward_path.push_back(node);
//     }
    
//     // Generate simple return path using TreeManager directly
//     TreeManager tree_manager(home_config, goal_config, 1000, 1000, 1000, 0.5, 2.0);
//     // auto return_path = tree_manager.createReturnPathSimple(forward_path);
//     auto return_path = createReturnPathSimple(forward_path);
    
//     // Verify return path has exactly the same number of nodes
//     ASSERT_EQ(return_path.size(), forward_path.size());
    
//     // Print out the paths to verify
//     std::cout << "Forward path:" << std::endl;
//     for (size_t i = 0; i < forward_path.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << forward_path[i]->q[j];
//             if (j < 5) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     std::cout << "Return path:" << std::endl;
//     for (size_t i = 0; i < return_path.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << return_path[i]->q[j];
//             if (j < 5) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     // Verify first node of return path matches last node of forward path
//     for (int j = 0; j < 6; j++) {
//         EXPECT_NEAR(return_path[0]->q[j], forward_path.back()->q[j], 1e-6);
//     }
    
//     // Verify last node of return path matches first node of forward path
//     for (int j = 0; j < 6; j++) {
//         EXPECT_NEAR(return_path.back()->q[j], forward_path[0]->q[j], 1e-6);
//     }
    
//     // Verify intermediate nodes are also reversed
//     for (size_t i = 1; i < return_path.size() - 1; i++) {
//         for (int j = 0; j < 6; j++) {
//             EXPECT_NEAR(return_path[i]->q[j], 
//                       forward_path[forward_path.size() - 1 - i]->q[j], 1e-6);
//         }
//     }
// }


// TEST(RRTStarTests, SimpleReturnPathCreation) {
//     auto home_config = getValidJointConfig(true);
//     auto goal_config = getValidJointConfig(false);
    
//     // Create a manual forward path
//     std::vector<std::shared_ptr<Node>> forward_path;
    
//     // Add waypoints from home to goal with proper interpolation
//     std::vector<std::array<double, 6>> waypoints;
//     waypoints.push_back(home_config); // Start
    
//     // Create two intermediate points at 1/3 and 2/3 of the way
//     for (int step = 1; step <= 2; step++) {
//         std::array<double, 6> intermediate_config = home_config;
//         double t = step / 3.0; // 1/3 and 2/3
        
//         for (int j = 0; j < 6; j++) {
//             intermediate_config[j] = home_config[j] + t * (goal_config[j] - home_config[j]);
//         }
//         waypoints.push_back(intermediate_config);
//     }
    
//     waypoints.push_back(goal_config); // Goal
    
//     // Create nodes and build forward path
//     for (size_t i = 0; i < waypoints.size(); i++) {
//         auto node = std::make_shared<Node>(waypoints[i]);
//         if (i > 0) {
//             node->parent = forward_path.back();
//         }
//         forward_path.push_back(node);
//     }
    
//     // Export forward path to YAML
//     std::string yaml_file = "test_forward_path.yaml";
//     bool export_success = exportPathToYaml(forward_path, yaml_file);
//     EXPECT_TRUE(export_success);
    
//     std::cout << "Forward path exported to YAML file" << std::endl;
    
//     // Create return path using the YAML file
//     auto return_path_from_yaml = createReturnPathFromYaml(yaml_file);
    
//     // Also test the direct method for comparison
//     auto return_path_simple = createReturnPathSimple(forward_path);
    
//     // Verify both return paths have the correct size
//     ASSERT_EQ(return_path_from_yaml.size(), forward_path.size());
//     ASSERT_EQ(return_path_simple.size(), forward_path.size());
    
//     // Print out all paths to verify
//     std::cout << "Forward path:" << std::endl;
//     for (size_t i = 0; i < forward_path.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << forward_path[i]->q[j];
//             if (j < 5) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     std::cout << "Return path from YAML:" << std::endl;
//     for (size_t i = 0; i < return_path_from_yaml.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << return_path_from_yaml[i]->q[j];
//             if (j < 5) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     std::cout << "Return path simple:" << std::endl;
//     for (size_t i = 0; i < return_path_simple.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << return_path_simple[i]->q[j];
//             if (j < 5) std::cout << ", ";
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     // Verify both return paths match the expected reverse of forward path
//     for (size_t i = 0; i < return_path_from_yaml.size(); i++) {
//         size_t forward_index = forward_path.size() - 1 - i;
//         for (int j = 0; j < 6; j++) {
//             // Check YAML-based return path
//             EXPECT_NEAR(return_path_from_yaml[i]->q[j], forward_path[forward_index]->q[j], 1e-6);
            
//             // Also verify simple return path
//             EXPECT_NEAR(return_path_simple[i]->q[j], forward_path[forward_index]->q[j], 1e-6);
//         }
//     }
    
//     // Check if both return paths are consistent with each other
//     for (size_t i = 0; i < return_path_from_yaml.size(); i++) {
//         for (int j = 0; j < 6; j++) {
//             EXPECT_NEAR(return_path_from_yaml[i]->q[j], return_path_simple[i]->q[j], 1e-6);
//         }
//     }
    
//     // Optional: Clean up the temporary YAML file
//     // std::remove(yaml_file.c_str());
// }

TEST(RRTStarTests, SimpleReturnPathCreation) {
    auto home_config = getValidJointConfig(true);
    auto goal_config = getValidJointConfig(false);
    
    // Create a planner with the RRT* algorithm
    RRTStarModified planner(
        home_config,
        goal_config,
        WorkspaceLimits::WIDTH,
        WorkspaceLimits::HEIGHT,
        WorkspaceLimits::DEPTH,
        0.5, 2.0, 0.1, 2000,
        WorkspaceLimits::MIN_X,
        WorkspaceLimits::MIN_Y,
        WorkspaceLimits::MIN_Z
    );
    
    // Generate a path using the actual RRT* algorithm
    auto forward_path = planner.findPath();
    
    // Ensure a path was found
    ASSERT_FALSE(forward_path.empty()) << "RRT* failed to find a path";
    std::cout << "Generated path with " << forward_path.size() << " waypoints" << std::endl;
    
    // Export forward path to YAML
    std::string yaml_file = "test_forward_path.yaml";
    bool export_success = exportPathToYaml(forward_path, yaml_file);
    EXPECT_TRUE(export_success);
    
    std::cout << "Forward path exported to YAML file" << std::endl;
    
    // Create return path using the YAML file
    auto return_path_from_yaml = createReturnPathFromYaml(yaml_file);
    
    // Also test the direct method for comparison
    auto return_path_simple = createReturnPathSimple(forward_path);
    
    // Verify both return paths have the correct size
    ASSERT_EQ(return_path_from_yaml.size(), forward_path.size());
    ASSERT_EQ(return_path_simple.size(), forward_path.size());
    
    // Print out all paths to verify
    std::cout << "Forward path:" << std::endl;
    for (size_t i = 0; i < forward_path.size(); i++) {
        std::cout << "Node " << i << ": [";
        for (int j = 0; j < 6; j++) {
            std::cout << forward_path[i]->q[j];
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    // Print return paths as well...
    
    // Verify both return paths match the expected reverse of forward path
    for (size_t i = 0; i < return_path_from_yaml.size(); i++) {
        size_t forward_index = forward_path.size() - 1 - i;
        for (int j = 0; j < 6; j++) {
            // Check YAML-based return path
            EXPECT_NEAR(return_path_from_yaml[i]->q[j], forward_path[forward_index]->q[j], 1e-6);
            
            // Also verify simple return path
            EXPECT_NEAR(return_path_simple[i]->q[j], forward_path[forward_index]->q[j], 1e-6);
        }
    }
    
    // Optional: Clean up the temporary YAML file
    // std::remove(yaml_file.c_str());
}



// #include "rrtstar_main.h"
// #include "robot_kinematics.h"
// #include <gtest/gtest.h>
// #include "path_export.h"
// #include "path_return.h"
// #include <cstdio>  // For std::remove

// // The original tests remain unchanged.

// const std::vector<std::array<double, 6>> reference_path = {
//     {0, 0, 0, 0, 0, 0},
//     {1, 1, 1, 0, 0, 0},
//     {2, 2, 2, 0, 0, 0},
//     {10, 10, 10, 0, 0, 0}
// };

// void printPosition(const std::array<double, 6>& q) {
//     auto T = RobotKinematics::computeFK(q);
//     Eigen::Vector3d pos = T.translation();
//     std::cout << "Position for config " << q[0] << "," << q[1] << "," << q[2] << ","
//               << q[3] << "," << q[4] << "," << q[5] << " is: ("
//               << pos.x() << ", " << pos.y() << ", " << pos.z() << ")" << std::endl;
// }

// std::array<double, 6> getValidJointConfig(bool start_config = true) {
//     if (start_config) {
//         return {0.0, 0.0, 0.0, 0.0, 0.0, 0.0}; // Home position
//     } else {
//         return {0.1, 0.3, 0.0, 0.0, 0.1, 0.0};
//     }
// }

// TEST(RRTStarTests, IsStateValid) {
//     RRTStarModified::updateObstacles({});
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     printPosition(valid_start);
    
//     RRTStarModified planner(
//         valid_start,    
//         valid_goal,     
//         1000, 1000, 1000, 10, 50, 10, 1000, -500, -500, -500
//     );
    
//     CollisionDetection collision(10.0);
//     EXPECT_TRUE(collision.isStateValid(valid_start));
    
//     std::array<double, 6> invalid_q = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
//     EXPECT_FALSE(collision.isStateValid(invalid_q));
// }

// TEST(RRTStarTests, LineAABBIntersection) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     RRTStarModified planner(
//         valid_start,    
//         valid_goal,     
//         1000, 1000, 1000, 10, 50, 10, 1000, -500, -500, -500
//     );
    
//     CollisionDetection collision(10.0);
    
//     std::array<double, 3> start = {0, 0, 0};
//     std::array<double, 3> end = {5, 5, 5};
//     std::array<double, 3> box_min = {1, 1, 1};
//     std::array<double, 3> box_max = {4, 4, 4};

//     EXPECT_TRUE(collision.lineAABBIntersection(start, end, box_min, box_max));
// }

// TEST(RRTStarTests, AdaptiveGoalBias) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     TreeManager tree_manager(
//         valid_start,    
//         valid_goal,     
//         1000, 1000, 1000, 9.99, 50, -500, -500, -500
//     );
    
//     int goal_samples = 0;
//     for (int i = 0; i < 1000; ++i) {
//         auto random_node = tree_manager.getRandomNode(i, 1000);
//         if (random_node->q == tree_manager.getGoalConfig()) {
//             goal_samples++;
//         }
//     }
//     EXPECT_GT(goal_samples, 100);
// }

// TEST(RRTStarTests, DynamicInterpolationSteps) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     RRTStarModified planner(
//         valid_start,    
//         valid_goal,     
//         1000, 1000, 1000, 9.99, 50, 10, 1000, -500, -500, -500
//     );
    
//     auto node1 = std::make_shared<Node>(valid_start);
//     auto node2 = std::make_shared<Node>(valid_goal);

//     CollisionDetection collision(10.0);
//     PathOptimization path_optimizer(collision, 9.99);
    
//     auto [collision_free, steps] = path_optimizer.isCollisionFree(node1, node2);
//     EXPECT_GE(steps, 10);
// }

// TEST(RRTStarTests, PathSmoothing) {
//     std::array<double, 6> start_config = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
//     std::array<double, 6> goal_config = {0.1, 0.1, 0.0, 0.0, 0.0, 0.0};
    
//     printPosition(start_config);
//     printPosition(goal_config);
    
//     RRTStarModified planner(
//         start_config,
//         goal_config,
//         1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
//     );
    
//     planner.setVisualizationEnabled(false);
    
//     std::vector<std::shared_ptr<Node>> path;
//     auto start_node = std::make_shared<Node>(start_config);
    
//     std::array<double, 6> mid_config = start_config;
//     for (int i = 0; i < 6; i++) {
//         mid_config[i] = start_config[i] + 0.5 * (goal_config[i] - start_config[i]);
//     }
//     auto mid_node = std::make_shared<Node>(mid_config);
    
//     auto goal_node = std::make_shared<Node>(goal_config);
    
//     mid_node->parent = start_node;
//     goal_node->parent = mid_node;
    
//     path.push_back(start_node);
//     path.push_back(mid_node);
//     path.push_back(goal_node);
    
//     std::cout << "Created manual path with " << path.size() << " nodes" << std::endl;
    
//     ASSERT_GT(path.size(), 0) << "Path creation failed";
    
//     CollisionDetection collision(0.1);
//     PathOptimization path_optimizer(collision, 0.5);
    
//     path_optimizer.optimizePath(path);
//     std::cout << "Optimized path size: " << path.size() << std::endl;
    
//     std::cout << "Optimized path nodes:" << std::endl;
//     for (size_t i = 0; i < path.size(); ++i) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; ++j) {
//             std::cout << path[i]->q[j] << (j < 5 ? ", " : "");
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     const double max_step = PathOptimization::MAX_JOINT_VEL / PathOptimization::CONTROL_RATE * 1.1;
    
//     for (size_t i = 1; i < path.size(); ++i) {
//         double dist = TreeManager::distance(path[i-1], path[i]);
//         std::cout << "Step " << i << " distance: " << dist << std::endl;
//         EXPECT_LE(dist, max_step) << "Step " << i << " exceeds velocity constraint";
//     }
    
//     double total_path_length = 0.0;
//     for (size_t i = 1; i < path.size(); ++i) {
//         total_path_length += TreeManager::distance(path[i-1], path[i]);
//     }
//     std::cout << "Total path length: " << total_path_length << std::endl;
    
//     EXPECT_NEAR(path.front()->q[0], start_config[0], 1e-6);
//     EXPECT_NEAR(path.front()->q[1], start_config[1], 1e-6);
//     EXPECT_NEAR(path.back()->q[0], goal_config[0], 1e-6);
//     EXPECT_NEAR(path.back()->q[1], goal_config[1], 1e-6);
// }

// TEST(RRTStarTests, RegressionTest) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     RRTStarModified planner(
//         valid_start,    
//         valid_goal,     
//         1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
//     );
    
//     planner.setVisualizationEnabled(false);
    
//     std::vector<std::shared_ptr<Node>> path;
//     std::vector<std::array<double, 6>> waypoints = { valid_start,
//         {0.03, 0.03, 0.0, 0, 0.03, 0},
//         {0.06, 0.06, 0.0, 0, 0.06, 0},
//         valid_goal
//     };
    
//     for (const auto& waypoint : waypoints) {
//         path.push_back(std::make_shared<Node>(waypoint));
//     }
    
//     for (size_t i = 1; i < path.size(); ++i) {
//         path[i]->parent = path[i-1];
//     }
    
//     std::cout << "Original path size: " << path.size() << std::endl;
    
//     CollisionDetection collision(0.1);
//     PathOptimization path_optimizer(collision, 0.5);
    
//     path_optimizer.optimizePath(path);
//     std::cout << "Optimized path size: " << path.size() << std::endl;
    
//     for (size_t i = 1; i < path.size(); ++i) {
//         double dist = TreeManager::distance(path[i-1], path[i]);
//         std::cout << "Step " << i << " distance: " << dist << std::endl;
//         EXPECT_LE(dist, 0.6) << "Step " << i << " exceeds maximum step size";
//     }
// }

// TEST(RRTStarTests, ExportPathForRobot) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     RRTStarModified planner(
//         valid_start,    
//         valid_goal,     
//         WorkspaceLimits::WIDTH,
//         WorkspaceLimits::HEIGHT,
//         WorkspaceLimits::DEPTH,
//         0.5, 2.0, 0.1, 2000, 
//         WorkspaceLimits::MIN_X,
//         WorkspaceLimits::MIN_Y,
//         WorkspaceLimits::MIN_Z
//     );
    
//     std::vector<std::shared_ptr<Node>> path;
//     auto start_node = std::make_shared<Node>(valid_start);
    
//     std::array<double, 6> mid_config1 = valid_start;
//     std::array<double, 6> mid_config2 = valid_start;
//     for (int i = 0; i < 6; i++) {
//         double diff = valid_goal[i] - valid_start[i];
//         mid_config1[i] = valid_start[i] + 0.33 * diff;
//         mid_config2[i] = valid_start[i] + 0.66 * diff;
//     }
    
//     auto mid_node1 = std::make_shared<Node>(mid_config1);
//     auto mid_node2 = std::make_shared<Node>(mid_config2);
//     auto goal_node = std::make_shared<Node>(valid_goal);
    
//     mid_node1->parent = start_node;
//     mid_node2->parent = mid_node1;
//     goal_node->parent = mid_node2;
    
//     path.push_back(start_node);
//     path.push_back(mid_node1);
//     path.push_back(mid_node2);
//     path.push_back(goal_node);
    
//     std::vector<std::array<double, 8>> robot_commands;
//     exportPathForRobot(path, robot_commands);
    
//     std::cout << "Robot commands for path:" << std::endl;
//     for (size_t i = 0; i < robot_commands.size(); i++) {
//         std::cout << "Command " << i << ": [";
//         for (int j = 0; j < 8; j++) {
//             std::cout << robot_commands[i][j] << (j < 7 ? ", " : "");
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     ASSERT_EQ(robot_commands.size(), path.size());
    
//     for (size_t i = 0; i < path.size(); i++) {
//         for (int j = 0; j < 6; j++) {
//             EXPECT_NEAR(robot_commands[i][j], path[i]->q[j], 1e-6);
//         }
//     }
    
//     for (const auto& cmd : robot_commands) {
//         EXPECT_GE(cmd[6], 0.1);
//         EXPECT_LE(cmd[6], 1.0);
//         EXPECT_GE(cmd[7], 0.1);
//         EXPECT_LE(cmd[7], 1.0);
//     }
    
//     EXPECT_NEAR(robot_commands[0][6], 0.5, 1e-6);
//     EXPECT_NEAR(robot_commands[0][7], 0.3, 1e-6);
// }

// TEST(RRTStarTests, ExportEmptyPath) {
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
    
//     CollisionDetection collision(0.1);
//     PathOptimization path_optimizer(collision, 0.5);
    
//     std::vector<std::shared_ptr<Node>> empty_path;
    
//     std::vector<std::array<double, 8>> robot_commands;
//     exportPathForRobot(empty_path, robot_commands);
    
//     EXPECT_TRUE(robot_commands.empty());
// }

// TEST(RRTStarTests, SimpleReturnPathCreation) {
//     auto home_config = getValidJointConfig(true);
//     auto goal_config = getValidJointConfig(false);
    
//     std::vector<std::shared_ptr<Node>> forward_path;
    
//     std::vector<std::array<double, 6>> waypoints = {
//         home_config,
//         {0.033, 0.033, 0.0, 0, 0.033, 0},
//         {0.066, 0.066, 0.0, 0, 0.066, 0},
//         goal_config
//     };
    
//     for (size_t i = 0; i < waypoints.size(); i++) {
//         auto node = std::make_shared<Node>(waypoints[i]);
//         if (i > 0) {
//             node->parent = forward_path.back();
//         }
//         forward_path.push_back(node);
//     }
    
//     // Using the original simple reversal function.
//     auto return_path = createReturnPathSimple(forward_path);
//     ASSERT_EQ(return_path.size(), forward_path.size());
    
//     std::cout << "Forward path:" << std::endl;
//     for (size_t i = 0; i < forward_path.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << forward_path[i]->q[j] << (j < 5 ? ", " : "");
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     std::cout << "Return path:" << std::endl;
//     for (size_t i = 0; i < return_path.size(); i++) {
//         std::cout << "Node " << i << ": [";
//         for (int j = 0; j < 6; j++) {
//             std::cout << return_path[i]->q[j] << (j < 5 ? ", " : "");
//         }
//         std::cout << "]" << std::endl;
//     }
    
//     for (int j = 0; j < 6; j++) {
//         EXPECT_NEAR(return_path[0]->q[j], forward_path.back()->q[j], 1e-6);
//     }
    
//     for (size_t i = 1; i < return_path.size() - 1; i++) {
//         for (int j = 0; j < 6; j++) {
//             EXPECT_NEAR(return_path[i]->q[j], 
//                       forward_path[forward_path.size() - 1 - i]->q[j], 1e-6);
//         }
//     }
// }

// // New test for YAML-based functionality:
// TEST(RRTStarTests, YamlExportLoadReturnPath) {
//     // Create a simple forward path with start, intermediate, and goal nodes.
//     auto valid_start = getValidJointConfig(true);
//     auto valid_goal = getValidJointConfig(false);
//     std::vector<std::shared_ptr<Node>> forward_path;
    
//     auto start_node = std::make_shared<Node>(valid_start);
    
//     std::array<double, 6> mid_config;
//     for (int i = 0; i < 6; i++) {
//         mid_config[i] = valid_start[i] + 0.5 * (valid_goal[i] - valid_start[i]);
//     }
//     auto mid_node = std::make_shared<Node>(mid_config);
//     mid_node->parent = start_node;
    
//     auto goal_node = std::make_shared<Node>(valid_goal);
//     goal_node->parent = mid_node;
    
//     forward_path.push_back(start_node);
//     forward_path.push_back(mid_node);
//     forward_path.push_back(goal_node);
    
//     // Export the forward path to a YAML file.
//     std::string yaml_file = "temp_path.yaml";
//     bool export_success = exportPathToYaml(forward_path, yaml_file);
//     EXPECT_TRUE(export_success);
    
//     // Load the robot commands from the YAML file.
//     std::vector<std::array<double, 8>> robot_commands;
//     bool load_success = loadPathFromYaml(yaml_file, robot_commands);
//     EXPECT_TRUE(load_success);
//     EXPECT_EQ(robot_commands.size(), forward_path.size());
    
//     // Verify that the joint values match.
//     for (size_t i = 0; i < forward_path.size(); ++i) {
//         for (int j = 0; j < 6; ++j) {
//             EXPECT_NEAR(robot_commands[i][j], forward_path[i]->q[j], 1e-6);
//         }
//     }
    
//     // Create a return path using the YAML file.
//     auto return_path = createReturnPathFromYaml(yaml_file);
//     EXPECT_EQ(return_path.size(), forward_path.size());
    
//     // Verify that the return path is the reverse of the forward path.
//     for (size_t i = 0; i < return_path.size(); ++i) {
//         for (int j = 0; j < 6; ++j) {
//             EXPECT_NEAR(return_path[i]->q[j], forward_path[forward_path.size()-1-i]->q[j], 1e-6);
//         }
//     }
    
//     // Clean up the temporary YAML file.
//     // std::remove(yaml_file.c_str());
// }
