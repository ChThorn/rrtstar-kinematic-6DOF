#include <gtest/gtest.h>
#include "hand_eye_calibration.h"
#include <Eigen/Dense>

// Helper function to create a homogeneous transformation matrix from roll-pitch-yaw angles and translation
Eigen::Matrix4d createPose(double roll, double pitch, double yaw, double x, double y, double z) {
    Eigen::AngleAxisd rollAngle(roll * M_PI / 180, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd pitchAngle(pitch * M_PI / 180, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd yawAngle(yaw * M_PI / 180, Eigen::Vector3d::UnitZ());
    Eigen::Quaterniond q = yawAngle * pitchAngle * rollAngle;
    Eigen::Matrix4d pose = Eigen::MatrixXd::Identity(4, 4);
    pose.topLeftCorner(3, 3) = q.toRotationMatrix();
    pose.topRightCorner(3, 1) << x, y, z;
    return pose;
}

// Test case: Simple scenario with identity poses
TEST(HandEyeCalibrationTest, IdentityPoses) {
    std::vector<Eigen::Matrix4d> bHg = {Eigen::MatrixXd::Identity(4, 4)};
    std::vector<Eigen::Matrix4d> cHw = {Eigen::MatrixXd::Identity(4, 4)};

    Eigen::Matrix4d gHc = HandEyeCalibration::handEye(bHg, cHw);
    EXPECT_TRUE(gHc.isApprox(Eigen::MatrixXd::Identity(4, 4), 1e-6)); // Expect identity transformation
}

// Test case: Simulated poses with known ground truth
TEST(HandEyeCalibrationTest, KnownGroundTruth) {
    // Define robot poses (bHg)
    std::vector<Eigen::Matrix4d> bHg = {
        createPose(0, 0, 0, 0, 0, 0),
        createPose(30, 45, 60, 100, 200, 300)
    };

    // Define camera poses (cHw)
    std::vector<Eigen::Matrix4d> cHw = {
        createPose(0, 0, 0, 0, 0, 0),
        createPose(-30, -45, -60, -100, -200, -300)
    };

    // Compute hand-eye transformation
    Eigen::Matrix4d gHc = HandEyeCalibration::handEye(bHg, cHw);

    // Expected result: gHc should align the two sets of poses
    Eigen::Matrix4d expected_gHc = createPose(0, 0, 0, 0, 0, 0); // Adjust based on your ground truth
    EXPECT_TRUE(gHc.isApprox(expected_gHc, 1e-6));
}

// Test case: Validate calibration function with real-world data
TEST(HandEyeCalibrationTest, RealWorldData) {
    Eigen::Matrix4d gHc;
    std::string path = "/home/thornch/Documents/Cpp/PathPlanning/rrtstar-kinematic-6DOF/data"; // Update with your data path

    int result = HandEyeCalibration::handEye_calib(gHc, path);
    ASSERT_EQ(result, 0); // Ensure calibration succeeds

    // Validate the computed gHc (example validation logic)
    std::vector<Eigen::Matrix4d> bHg = {
        createPose(0, 0, 0, 0, 0, 0),
        createPose(30, 45, 60, 100, 200, 300)
    };
    std::vector<Eigen::Matrix4d> cHw = {
        createPose(0, 0, 0, 0, 0, 0),
        createPose(-30, -45, -60, -100, -200, -300)
    };

    double total_error = 0.0;
    for (size_t i = 0; i < bHg.size(); ++i) {
        Eigen::Matrix4d computed_cHw = bHg[i] * gHc;
        total_error += (computed_cHw - cHw[i]).norm();
    }
    double avg_error = total_error / bHg.size();
    EXPECT_LT(avg_error, 1e-3); // Ensure average error is within tolerance
}