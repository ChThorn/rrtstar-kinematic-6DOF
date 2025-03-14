#ifndef PATH_RETURN_H
#define PATH_RETURN_H

#include <vector>
#include <memory>
#include <array>
#include <string>

// #include "tree_management.h"  // For Node definition
#include "path_export.h"      // For loading YAML files

// Forward declarations
struct Node;

/**
 * Creates a simple return path by reversing the forward path.
 * Each node in the return path copies the configuration from the corresponding node
 * in the forward path, and establishes parent relationships in reverse order.
 * 
 * @param forward_path The original path from start to goal
 * @return A new path from goal to start
 */
std::vector<std::shared_ptr<Node>> createReturnPathSimple(
    const std::vector<std::shared_ptr<Node>>& forward_path);

/**
 * Creates a return path from a YAML file.
 * The path is loaded from the YAML file and then reversed to create a return path.
 * 
 * @param yaml_filepath The path to the YAML file containing the forward path
 * @return A new path representing the return path from goal to start
 */
std::vector<std::shared_ptr<Node>> createReturnPathFromYaml(
    const std::string& yaml_filepath);

/**
 * Creates a return path from robot commands.
 * The robot commands are reversed to create a return path.
 * 
 * @param robot_commands The robot commands representing the forward path
 * @return A new path representing the return path from goal to start
 */
std::vector<std::shared_ptr<Node>> createReturnPathFromCommands(
    const std::vector<std::array<double, 8>>& robot_commands);

#endif // PATH_RETURN_H