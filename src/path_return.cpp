
#include "path_return.h"
#include <vector>
#include <memory>
#include <algorithm>
#include <iostream>

std::vector<std::shared_ptr<Node>> createReturnPathSimple(
    const std::vector<std::shared_ptr<Node>>& forward_path) {
    
    // Create a reversed copy of the forward path
    std::vector<std::shared_ptr<Node>> return_path;
    
    // Simply add nodes in reverse order
    for (auto it = forward_path.rbegin(); it != forward_path.rend(); ++it) {
        auto node = std::make_shared<Node>((*it)->q);
        
        if (!return_path.empty()) {
            node->parent = return_path.back();
        }
        
        return_path.push_back(node);
    }
    
    return return_path;
}

std::vector<std::shared_ptr<Node>> createReturnPathFromYaml(
    const std::string& yaml_filepath) {
    
    // Load robot commands from YAML file
    std::vector<std::array<double, 8>> robot_commands;
    
    if (!loadPathFromYaml(yaml_filepath, robot_commands)) {
        // If loading failed, return empty path
        std::cerr << "Failed to load path from YAML file: " << yaml_filepath << std::endl;
        return {};
    }
    
    // Create return path from the loaded robot commands
    return createReturnPathFromCommands(robot_commands);
}

std::vector<std::shared_ptr<Node>> createReturnPathFromCommands(
    const std::vector<std::array<double, 8>>& robot_commands) {
    
    // Create a reversed path from robot commands
    std::vector<std::shared_ptr<Node>> return_path;
    
    // Process commands in reverse order
    for (auto it = robot_commands.rbegin(); it != robot_commands.rend(); ++it) {
        // Extract joint values from the command
        std::array<double, 6> joint_values;
        for (int i = 0; i < 6; ++i) {
            joint_values[i] = (*it)[i];
        }
        
        // Create new node
        auto node = std::make_shared<Node>(joint_values);
        
        // Set parent if not the first node
        if (!return_path.empty()) {
            node->parent = return_path.back();
        }
        
        // Add to return path
        return_path.push_back(node);
    }
    
    // Print summary
    std::cout << "Created return path with " << return_path.size() << " waypoints" << std::endl;
    
    return return_path;
}