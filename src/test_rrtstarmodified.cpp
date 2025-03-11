#include "rrtstarmodified.h"
#include <gtest/gtest.h>

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

// TEST(RRTStarTests, IsStateValid) {
//     printPosition({0, 0, 0, 0, 0, 0});
    
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},
//         {1, 1, 1, 0, 0, 0},
//         1000,
//         1000,
//         1000,
//         10,
//         50,
//         10,
//         1000,
//         -500,
//         -500,
//         -500
//     );
    
//     std::array<double, 6> valid_q = {0, 0, 0, 0, 0, 0};
//     std::array<double, 6> invalid_q = {100, 100, 100, 100, 100, 100}; // Out of joint limits

//     // For now, expect both to be valid, and update this when joint limit checks are added.
//     EXPECT_TRUE(planner.isStateValid(valid_q));
//     EXPECT_TRUE(planner.isStateValid(invalid_q)); 
// }

// TEST(RRTStarTests, IsStateValid) {

//     RRTStarModified::updateObstacles({});
//     printPosition({0, 0, 0, 0, 0, 0});
    
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},
//         {1, 1, 1, 0, 0, 0},
//         1000,
//         1000,
//         1000,
//         10,
//         50,
//         10,
//         1000,
//         -500,
//         -500,
//         -500
//     );
    
//     std::array<double, 6> valid_q = {0, 0, 0, 0, 0, 0};
//     std::array<double, 6> invalid_q = {100, 100, 100, 100, 100, 100}; // Out of joint limits

//     // // For now, expect both to be valid, and update this when joint limit checks are added.
//     // EXPECT_TRUE(planner.isStateValid(valid_q));
//     // EXPECT_TRUE(planner.isStateValid(invalid_q)); 

//     // Joint limits should now be enforced
//     EXPECT_TRUE(planner.isStateValid(valid_q));
//     EXPECT_FALSE(planner.isStateValid(invalid_q));  // Changed expectation
// }

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
    
    // Test valid configuration
    EXPECT_TRUE(planner.isStateValid(valid_start));
    
    // Test invalid configuration (out of joint limits)
    std::array<double, 6> invalid_q = {10.0, 10.0, 10.0, 10.0, 10.0, 10.0};
    EXPECT_FALSE(planner.isStateValid(invalid_q));
}



TEST(RRTStarTests, LineAABBIntersection) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 10, 50, 10, 1000, -500, -500, -500
    );
    // RRTStarModified planner(
    //     {0, 0, 0, 0, 0, 0},         // start config
    //     {1, 1, 1, 0, 0, 0},          // goal config
    //     1000,                        // map width
    //     1000,                        // map height
    //     1000,                        // map depth
    //     10,                          // step size
    //     50,                          // neighbor radius 
    //     10,                          // safety margin
    //     1000,                        // max iterations
    //     -500,                        // min_x
    //     -500,                        // min_y
    //     -500                         // min_z - adjusted to allow negative values
    // );
    
    std::array<double, 3> start = {0, 0, 0};
    std::array<double, 3> end = {5, 5, 5};
    std::array<double, 3> box_min = {1, 1, 1};
    std::array<double, 3> box_max = {4, 4, 4};

    EXPECT_TRUE(planner.testLineAABBIntersection(start, end, box_min, box_max));
}

// TEST(RRTStarTests, AdaptiveGoalBias) {
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},         // start config
//         {1, 1, 1, 0, 0, 0},          // goal config
//         1000,                        // map width
//         1000,                        // map height
//         1000,                        // map depth
//         9.99,                        // step size
//         50,                          // neighbor radius 
//         10,                          // safety margin
//         1000,                        // max iterations
//         -500,                        // min_x
//         -500,                        // min_y
//         -500                         // min_z - adjusted to allow negative values
//     );
    
//     int goal_samples = 0;
//     for (int i = 0; i < 1000; ++i) {
//         auto random_node = planner.getRandomNode(i); // Pass the iteration count
//         if (random_node->q == planner.getGoalConfig()) { // Use a getter for goal_config
//             goal_samples++;
//         }
//     }
//     EXPECT_GT(goal_samples, 100); // Expect at least 10% goal samples
// }

TEST(RRTStarTests, AdaptiveGoalBias) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 9.99, 50, 10, 1000, -500, -500, -500
    );
    
    int goal_samples = 0;
    for (int i = 0; i < 1000; ++i) {
        auto random_node = planner.getRandomNode(i); // Pass the iteration count
        if (random_node->q == planner.getGoalConfig()) { // Use a getter for goal_config
            goal_samples++;
        }
    }
    EXPECT_GT(goal_samples, 100); // Expect at least 10% goal samples
}

// TEST(RRTStarTests, DynamicInterpolationSteps) {
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},         // start config
//         {1, 1, 1, 0, 0, 0},          // goal config
//         1000,                        // map width
//         1000,                        // map height
//         1000,                        // map depth
//         9.99,                        // step size
//         50,                          // neighbor radius 
//         10,                          // safety margin
//         1000,                        // max iterations
//         -500,                        // min_x
//         -500,                        // min_y
//         -500                         // min_z - adjusted to allow negative values
//     );
    
//     auto node1 = std::make_shared<Node>(std::array<double, 6>{0, 0, 0, 0, 0, 0});
//     auto node2 = std::make_shared<Node>(std::array<double, 6>{1, 1, 1, 0, 0, 0});

//     int steps = planner.testIsCollisionFree(node1, node2);
//     EXPECT_GE(steps, 10); // Ensure at least 10 steps for large distances
// }

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

    int steps = planner.testIsCollisionFree(node1, node2);
    EXPECT_GE(steps, 10); // Ensure at least 10 steps for large distances
}

// TEST(RRTStarTests, PathSmoothing) {
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},         // start
//         {0.3, 0.3, 0.3, 0, 0, 0},    // goal
//         1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
//     );
    
//     planner.setVisualizationEnabled(false);
    
//     // INSTEAD OF CALLING GLOBAL PLANNER, CREATE A SIMPLE PATH MANUALLY
//     std::vector<std::shared_ptr<Node>> path;
    
//     // Create a simple direct path from start to goal
//     auto start_node = std::make_shared<Node>(std::array<double, 6>{0, 0, 0, 0, 0, 0});
//     auto mid_node = std::make_shared<Node>(std::array<double, 6>{0.15, 0.15, 0.15, 0, 0, 0});
//     auto goal_node = std::make_shared<Node>(std::array<double, 6>{0.3, 0.3, 0.3, 0, 0, 0});
    
//     // Set up parent relationships
//     mid_node->parent = start_node;
//     goal_node->parent = mid_node;
    
//     // Add to path
//     path.push_back(start_node);
//     path.push_back(mid_node);
//     path.push_back(goal_node);
    
//     std::cout << "Created manual path with " << path.size() << " nodes" << std::endl;
    
//     // Verify basic properties of the path
//     ASSERT_GT(path.size(), 0) << "Path creation failed";
    
//     // Apply optimization
//     planner.optimizePath(path);
//     std::cout << "Optimized path size: " << path.size() << std::endl;
    
//     // Verify velocity constraints on the path
//     for(size_t i = 1; i < path.size(); ++i) {
//         double dist = planner.distance(path[i-1], path[i]);
//         EXPECT_LE(dist, RRTStarModified::MAX_JOINT_VEL/RRTStarModified::CONTROL_RATE * 1.1);
//     }
// }

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
    
    // INSTEAD OF CALLING GLOBAL PLANNER, CREATE A SIMPLE PATH MANUALLY
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
    
    // Apply optimization
    planner.optimizePath(path);
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
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = planner.distance(path[i-1], path[i]);
        std::cout << "Step " << i << " distance: " << dist << std::endl;
        EXPECT_LE(dist, RRTStarModified::MAX_JOINT_VEL/RRTStarModified::CONTROL_RATE * 1.1)
            << "Step " << i << " exceeds velocity constraint";
    }
    
    // Calculate path metrics
    double total_path_length = 0.0;
    for (size_t i = 1; i < path.size(); ++i) {
        total_path_length += planner.distance(path[i-1], path[i]);
    }
    std::cout << "Total path length: " << total_path_length << std::endl;
    
    // Ensure path still connects start to goal
    EXPECT_NEAR(path.front()->q[0], start_config[0], 1e-6);
    EXPECT_NEAR(path.front()->q[1], start_config[1], 1e-6);
    EXPECT_NEAR(path.back()->q[0], goal_config[0], 1e-6);
    EXPECT_NEAR(path.back()->q[1], goal_config[1], 1e-6);
}

// TEST(RRTStarTests, RegressionTest) {
//     RRTStarModified planner(
//         {0, 0, 0, 0, 0, 0},         // start config
//         {0.3, 0.3, 0.3, 0, 0, 0},    // goal config
//         1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
//     );
    
//     planner.setVisualizationEnabled(false);
    
//     // CREATE MANUAL PATH INSTEAD OF USING PLANNER
//     std::vector<std::shared_ptr<Node>> path;
    
//     // Create several nodes for a more realistic path
//     std::vector<std::array<double, 6>> waypoints = {
//         {0.0, 0.0, 0.0, 0, 0, 0},
//         {0.1, 0.1, 0.1, 0, 0, 0},
//         {0.2, 0.2, 0.15, 0, 0, 0},
//         {0.3, 0.3, 0.3, 0, 0, 0}
//     };
    
//     // Create nodes and build path
//     for (const auto& waypoint : waypoints) {
//         path.push_back(std::make_shared<Node>(waypoint));
//     }
    
//     // Set up parent relationships
//     for (size_t i = 1; i < path.size(); ++i) {
//         path[i]->parent = path[i-1];
//     }
    
//     std::cout << "Original path size: " << path.size() << std::endl;
    
//     // Apply optimization to get smaller steps
//     planner.optimizePath(path);
//     std::cout << "Optimized path size: " << path.size() << std::endl;
    
//     // Verify path properties
//     for (size_t i = 1; i < path.size(); ++i) {
//         double dist = planner.distance(path[i-1], path[i]);
//         std::cout << "Step " << i << " distance: " << dist << std::endl;
//         EXPECT_LE(dist, 0.6) << "Step " << i << " exceeds maximum step size";
//     }
// }

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
    
    // Apply optimization to get smaller steps
    planner.optimizePath(path);
    std::cout << "Optimized path size: " << path.size() << std::endl;
    
    // Verify path properties
    for (size_t i = 1; i < path.size(); ++i) {
        double dist = planner.distance(path[i-1], path[i]);
        std::cout << "Step " << i << " distance: " << dist << std::endl;
        EXPECT_LE(dist, 0.6) << "Step " << i << " exceeds maximum step size";
    }
}

// TEST(RRTStarTests, PrecisePoseMatching) {
//     Eigen::Isometry3d goal_pose = Eigen::Isometry3d::Identity();
//     goal_pose.translate(Eigen::Vector3d(0.5, 0.2, 0.8));
//     goal_pose.rotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
    
//     RRTStarModified planner(
//         Eigen::Isometry3d::Identity(), // Start at origin
//         goal_pose,
//         1.0, 1.0, 1.0,  // Map dimensions
//         0.1,             // Step size
//         0.5,             // Neighbor radius
//         0.05,            // Safety margin
//         1000             // Max iterations
//     );
    
//     auto path = planner.findPath();
//     ASSERT_FALSE(path.empty());
    
//     // Verify final pose
//     auto T_final = RobotKinematics::computeFK(path.back()->q);
//     Eigen::Vector3d pos_error = T_final.translation() - goal_pose.translation();
//     Eigen::Matrix3d rot_error = T_final.linear() * goal_pose.linear().transpose();
    
//     EXPECT_LT(pos_error.norm(), 0.02);      // 2cm position accuracy
//     EXPECT_LT(std::abs(Eigen::AngleAxisd(rot_error).angle()), 0.1); // <5.7 degrees
// }


TEST(RRTStarTests, ExportPathForRobot) {
    // Create a simple path similar to the PathSmoothing test
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,    // Valid start config
        valid_goal,     // Valid goal config
        1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
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
    
    // Export path for robot
    std::vector<std::array<double, 8>> robot_commands;
    planner.exportPathForRobot(path, robot_commands);
    
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

TEST(RRTStarTests, ExportEmptyPath) {
    auto valid_start = getValidJointConfig(true);
    auto valid_goal = getValidJointConfig(false);
    
    RRTStarModified planner(
        valid_start,
        valid_goal,
        1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500
    );
    
    // Create an empty path
    std::vector<std::shared_ptr<Node>> empty_path;
    
    // Export empty path
    std::vector<std::array<double, 8>> robot_commands;
    planner.exportPathForRobot(empty_path, robot_commands);
    
    // Verify result is also empty
    EXPECT_TRUE(robot_commands.empty());
}

TEST(RRTStarTests, SimpleReturnPathCreation) {
    auto home_config = getValidJointConfig(true);
    auto goal_config = getValidJointConfig(false);
    
    // Create a manual forward path
    std::vector<std::shared_ptr<Node>> forward_path;
    
    // Add waypoints from home to goal
    std::vector<std::array<double, 6>> waypoints = {
        home_config,                          // A
        {0.033, 0.033, 0.0, 0, 0.033, 0},     // C
        {0.066, 0.066, 0.0, 0, 0.066, 0},     // D
        goal_config                           // G
    };
    
    // Create nodes and build forward path
    for (size_t i = 0; i < waypoints.size(); i++) {
        auto node = std::make_shared<Node>(waypoints[i]);
        if (i > 0) {
            node->parent = forward_path.back();
        }
        forward_path.push_back(node);
    }
    
    // Generate simple return path
    RRTStarModified planner(home_config, goal_config, 1000, 1000, 1000, 0.5, 2.0, 0.1, 2000, -500, -500, -500);
    auto return_path = planner.createReturnPathSimple(forward_path);
    
    // Verify return path has exactly the same number of nodes
    ASSERT_EQ(return_path.size(), forward_path.size());
    
    // Print out the paths to verify
    std::cout << "Forward path:" << std::endl;
    for (size_t i = 0; i < forward_path.size(); i++) {
        std::cout << "Node " << i << ": [";
        for (int j = 0; j < 6; j++) {
            std::cout << forward_path[i]->q[j];
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    std::cout << "Return path:" << std::endl;
    for (size_t i = 0; i < return_path.size(); i++) {
        std::cout << "Node " << i << ": [";
        for (int j = 0; j < 6; j++) {
            std::cout << return_path[i]->q[j];
            if (j < 5) std::cout << ", ";
        }
        std::cout << "]" << std::endl;
    }
    
    // Verify first node of return path matches last node of forward path
    for (int j = 0; j < 6; j++) {
        EXPECT_NEAR(return_path[0]->q[j], forward_path.back()->q[j], 1e-6);
    }
    
    // Verify last node of return path matches first node of forward path
    for (int j = 0; j < 6; j++) {
        EXPECT_NEAR(return_path.back()->q[j], forward_path[0]->q[j], 1e-6);
    }
    
    // Verify intermediate nodes are also reversed
    for (size_t i = 1; i < return_path.size() - 1; i++) {
        for (int j = 0; j < 6; j++) {
            EXPECT_NEAR(return_path[i]->q[j], 
                      forward_path[forward_path.size() - 1 - i]->q[j], 1e-6);
        }
    }
}