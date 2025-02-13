#include <iostream>
#include <iomanip>
#include "forward_kinematics.h"

void printPoint(const ForwardKinematics::Point& point) {
    std::cout << std::fixed << std::setprecision(3);
    std::cout << "Position (x,y,z): ["
              << point[0] << "mm, "
              << point[1] << "mm, "
              << point[2] << "mm]" << std::endl;
    std::cout << "Orientation (rx,ry,rz): ["
              << point[3] * 180/M_PI << "°, "
              << point[4] * 180/M_PI << "°, "
              << point[5] * 180/M_PI << "°]" << std::endl;
}

int main() {
    ForwardKinematics fk;

    // Test case 1: Home position (all joints at zero)
    ForwardKinematics::Joint home_pose = {0, 0, 0, 0, 0, 0};
    std::cout << "Test case 1 - Home position:" << std::endl;
    auto result1 = fk.calculateFK(home_pose);
    printPoint(result1);

    // Test case 2: First joint rotated 90 degrees
    ForwardKinematics::Joint rotated_j1 = {M_PI/2, 0, 0, 0, 0, 0};
    std::cout << "\nTest case 2 - Joint 1 at 90 degrees:" << std::endl;
    auto result2 = fk.calculateFK(rotated_j1);
    printPoint(result2);

    // Test case 3: Reach forward position
    ForwardKinematics::Joint reach_forward = {0, -M_PI/4, -M_PI/4, 0, -M_PI/2, 0};
    std::cout << "\nTest case 3 - Reach forward position:" << std::endl;
    auto result3 = fk.calculateFK(reach_forward);
    printPoint(result3);

    return 0;
}