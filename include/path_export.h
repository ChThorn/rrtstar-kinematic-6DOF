#ifndef PATH_EXPORT_H
#define PATH_EXPORT_H

#include <vector>
#include <memory>
#include <array>
#include <cmath>  // For M_PI
#include <string> // For file paths
#include "tree_management.h" 

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

/**
 * Exports a path to a YAML file.
 * 
 * @param path The planned path as a sequence of nodes
 * @param filepath The path to the output YAML file
 * @return true if the export was successful, false otherwise
 */
bool exportPathToYaml(
    const std::vector<std::shared_ptr<Node>>& path,
    const std::string& filepath);

/**
 * Loads a path from a YAML file.
 * 
 * @param filepath The path to the input YAML file
 * @param robot_commands Output vector of robot commands loaded from the YAML file
 * @return true if the load was successful, false otherwise
 */
bool loadPathFromYaml(
    const std::string& filepath,
    std::vector<std::array<double, 8>>& robot_commands);

#endif // PATH_EXPORT_H