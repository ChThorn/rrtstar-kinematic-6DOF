#include <gtest/gtest.h>
#include "forward_kinematics.h"
#include "inverse_kinematics.h"
#include <Eigen/Dense>


TEST(FK_IK_Test, HomePosition) {
    ForwardKinematics fk;
    InverseKinematics ik;

    ForwardKinematics::Joint home_joints = {0, 0, 0, 0, 0, 0};
    auto tcp_pose = fk.calculateFK(home_joints);

    auto ik_solutions = ik.calculateMultipleIKSolutions(
        tcp_pose[0], tcp_pose[1], tcp_pose[2],
        tcp_pose[3] * 180 / M_PI, tcp_pose[4] * 180 / M_PI, tcp_pose[5] * 180 / M_PI);

    ASSERT_GT(ik_solutions.size(), 0); // Ensure at least one solution is found
}

TEST(FK_IK_Test, ExtremeAngles) {
    ForwardKinematics fk;
    InverseKinematics ik;

    ForwardKinematics::Joint extreme_joints = {-170, 120, -225, 180, -97, 180}; // Near joint limits
    auto tcp_pose = fk.calculateFK(extreme_joints);

    auto ik_solutions = ik.calculateMultipleIKSolutions(
        tcp_pose[0], tcp_pose[1], tcp_pose[2],
        tcp_pose[3] * 180 / M_PI, tcp_pose[4] * 180 / M_PI, tcp_pose[5] * 180 / M_PI);

    ASSERT_GT(ik_solutions.size(), 0); // Ensure at least one solution is found
}

TEST(FK_IK_Test, UnreachablePose) {
    InverseKinematics ik;

    double unreachable_x = 2000, unreachable_y = 2000, unreachable_z = 2000;
    double unreachable_rx = 0, unreachable_ry = 0, unreachable_rz = 0;

    auto ik_solutions = ik.calculateMultipleIKSolutions(
        unreachable_x, unreachable_y, unreachable_z,
        unreachable_rx, unreachable_ry, unreachable_rz);

    ASSERT_EQ(ik_solutions.size(), 0); // Ensure no solutions are found
}