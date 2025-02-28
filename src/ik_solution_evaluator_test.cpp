#include <gtest/gtest.h>
#include "inverse_kinematics.h"
#include "ik_solution_evaluator.h"
#include "obstack.h"
#include <opencv2/core.hpp>

class IKSolutionEvaluatorTest : public ::testing::Test {
protected:
    InverseKinematics ik;

    std::unique_ptr<IKSolutionEvaluator> createEvaluator(const std::array<double, 6>& current_joints) {
        return std::make_unique<IKSolutionEvaluator>(current_joints);
    }
};

TEST_F(IKSolutionEvaluatorTest, SelectsClosestToCurrentPosition) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    double x = 250.0, y = 0.0, z = 350.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);
    double best_total_movement = evaluator->getTotalMovement(best_solution);

    for (const auto& sol : solutions) {
        double current_movement = evaluator->getTotalMovement(sol);
        EXPECT_GE(current_movement, best_total_movement - 0.001)
            << "Should select solution with minimum total movement";
    }
}

TEST_F(IKSolutionEvaluatorTest, PreferredConfiguration) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    double x = 250.0, y = 0.0, z = 500.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;

    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);
    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    // Check feasibility of "UP" configuration
    bool has_up_solution = false;
    for (const auto& sol : solutions) {
        if (sol.configuration.elbow == "UP") {
            has_up_solution = true;
            break;
        }
    }
    if (!has_up_solution) {
        std::cout << "No UP configuration available; skipping test." << std::endl;
        return;
    }

    auto best_solution = evaluator->getBestSolution(solutions);

    EXPECT_EQ(best_solution.configuration.shoulder, "RIGHTY")
        << "Should prefer RIGHTY configuration";
    EXPECT_EQ(best_solution.configuration.elbow, "UP")
        << "Should prefer UP configuration";
}

TEST_F(IKSolutionEvaluatorTest, PrefersCenterJointRanges) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    double x = 300.0, y = 0.0, z = 400.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);
    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);

    std::array<int, 3> limited_joints = {1, 2, 4}; // Shoulder, elbow, wrist2
    for (int joint : limited_joints) {
        double angle = best_solution.joints[joint] * IKSolutionEvaluator::D2R;
        double min = InverseKinematics::joint_limits_min[joint];
        double max = InverseKinematics::joint_limits_max[joint];
        double mid = (min + max) / 2.0;
        double range = max - min;
        
        double distance_from_mid = std::abs(angle - mid);
        EXPECT_LT(distance_from_mid, range * 0.8) 
            << "Joint " << joint << " should be within reasonable range of center";
    }
}

TEST_F(IKSolutionEvaluatorTest, HandlesEmptySolutions) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    std::vector<IKSolution> empty_solutions;
    
    EXPECT_THROW(evaluator->getBestSolution(empty_solutions), std::runtime_error)
        << "Should throw exception for empty solutions";
}

TEST_F(IKSolutionEvaluatorTest, ManipulabilityCheck) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    // Define a pose that might have multiple solutions
    double x = 600.0, y = 0.0, z = 400.0;
    double rx = 60.0, ry = 0.0, rz = 0.0;
    
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);

    for (size_t i = 0; i < 5; ++i) {
        double angle_diff = std::abs(best_solution.joints[i] - best_solution.joints[i+1]);
        EXPECT_GT(angle_diff, 0.1) 
            << "Joints " << i << " and " << (i+1) << " too close to singular configuration";
    }
}

TEST_F(IKSolutionEvaluatorTest, HandlesExtremeJointAngles) {
    std::array<double, 6> current_joints = {90, 90, 90, 90, 90, 90};
    auto evaluator = createEvaluator(current_joints);

    double x = 100.0, y = 0.0, z = 100.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);
    for (size_t i = 0; i < 6; ++i) {
        double angle_rad = best_solution.joints[i] * IKSolutionEvaluator::D2R;
        EXPECT_GE(angle_rad, InverseKinematics::joint_limits_min[i])
            << "Joint " << i << " should not exceed minimum limit";
        EXPECT_LE(angle_rad, InverseKinematics::joint_limits_max[i])
            << "Joint " << i << " should not exceed maximum limit";
    }
}

TEST_F(IKSolutionEvaluatorTest, HandlesNearWorkspaceLimits) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    auto evaluator = createEvaluator(current_joints);

    double x = 1000.0, y = 0.0, z = 1000.0; // Near workspace limit
    double rx = 0.0, ry = 0.0, rz = 0.0;
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);
    EXPECT_GT(evaluator->getManipulabilityScore(best_solution.joints), 0.5)
        << "Solution should have reasonable manipulability near workspace limits";
}

TEST_F(IKSolutionEvaluatorTest, AvoidsObstacles) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    std::vector<Obstacle> obstacles = {
        Obstacle({300.0, 0.0, 400.0}, {50.0, 50.0, 50.0}) // Obstacle at (300, 0, 400) with size 50x50x50
    };
    auto evaluator = std::make_unique<IKSolutionEvaluator>(current_joints, obstacles);

    double x = 300.0, y = 0.0, z = 400.0;
    double rx = 0.0, ry = 0.0, rz = 0.0;
    auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

    ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

    auto best_solution = evaluator->getBestSolution(solutions);
    EXPECT_EQ(evaluator->getCollisionScore(best_solution), 1.0)
        << "Best solution should be collision-free";
}

// TEST_F(IKSolutionEvaluatorTest, DynamicObstacles) {
//     std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};

//     // Define a dynamic obstacle based on ROI and depth
//     cv::Rect roi(100, 100, 50, 50); // Example ROI
//     float depth_value = 300.0f;     // Example depth

//     // Create an obstacle with dynamic properties
//     Obstacle dynamic_obstacle(true); // Dynamic obstacle
//     dynamic_obstacle.updateFromDepthAndROI(roi, depth_value);

//     // Add the dynamic obstacle to the evaluator
//     std::vector<Obstacle> obstacles = {dynamic_obstacle};
//     auto evaluator = std::make_unique<IKSolutionEvaluator>(current_joints, obstacles);

//     // Define a pose near the obstacle
//     double x = 125.0, y = 125.0, z = 300.0; // Near the center of the ROI
//     double rx = 0.0, ry = 0.0, rz = 0.0;
//     auto solutions = ik.calculateMultipleIKSolutions(x, y, z, rx, ry, rz);

//     ASSERT_GT(solutions.size(), 0) << "Should find at least one IK solution";

//     // Get the best solution
//     auto best_solution = evaluator->getBestSolution(solutions);

//     // Ensure the solution avoids the dynamic obstacle
//     EXPECT_EQ(evaluator->getCollisionScore(best_solution), 1.0)
//         << "Best solution should be collision-free";
// }

TEST_F(IKSolutionEvaluatorTest, DynamicObstacles) {
    std::array<double, 6> current_joints = {0, 0, 0, 0, 0, 0};
    std::vector<Obstacle> obstacles = {Obstacle(true)}; // One dynamic obstacle

    IKSolutionEvaluator evaluator(current_joints, obstacles);

    // Define ROIs and depth values
    std::vector<cv::Rect> rois = {cv::Rect(100, 100, 50, 50)};
    std::vector<float> depth_values = {300.0f};

    // Update dynamic obstacles
    evaluator.updateDynamicObstacles(rois, depth_values);

    // Generate test IK solutions
    std::vector<IKSolution> solutions = {
        {{10.0, 10.0, 10.0, 0.0, 0.0, 0.0}, {"RIGHTY", "UP", "NOFLIP"}},
        {{110.0, 110.0, 300.0, 0.0, 0.0, 0.0}, {"RIGHTY", "UP", "NOFLIP"}}
    };

    // Evaluate best solution
    IKSolution best_solution = evaluator.getBestSolution(solutions);

    // Verify that the solution avoids the dynamic obstacle
    EXPECT_NE(best_solution.joints[0], 110.0);
    EXPECT_NE(best_solution.joints[1], 110.0);
    EXPECT_NE(best_solution.joints[2], 300.0);
}