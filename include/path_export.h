#ifndef PATH_EXPORT_H
#define PATH_EXPORT_H

#include <vector>
#include <memory>
#include <array>
#include <cmath>  // For M_PI

// Forward declarations
struct Node;

// Constants needed from PathOptimization
namespace PathExport {
    constexpr double MAX_JOINT_VEL = M_PI/4;  // rad/s (45 deg/s)
    constexpr double CONTROL_RATE = 100.0;    // Hz
}

/**
 * Exports a path as robot commands.
 * 
 * @param path The planned path as a sequence of nodes
 * @param robot_commands Output vector of robot commands:
 *        [0-5]: Joint values
 *        [6]: Speed (0.1-1.0)
 *        [7]: Acceleration (0.1-1.0)
 */
void exportPathForRobot(
    const std::vector<std::shared_ptr<Node>>& path,
    std::vector<std::array<double, 8>>& robot_commands);

#endif // PATH_EXPORT_H