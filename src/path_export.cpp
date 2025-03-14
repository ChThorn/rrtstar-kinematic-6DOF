#include "path_export.h"
#include <algorithm>  // For std::min, std::max
#include <cmath>      // For std::abs
#include <fstream>    // For file operations
#include <iostream>   // For error reporting
#include <yaml-cpp/yaml.h> // For YAML support

void exportPathForRobot(const std::vector<std::shared_ptr<Node>>& path,
                        std::vector<std::array<double, 8>>& robot_commands) 
{
    
    robot_commands.clear();
    
    // Handle empty path
    if (path.empty()) return;
    
    // Process each waypoint
    for (size_t i = 0; i < path.size(); i++) {
        std::array<double, 8> command;
        
        // Joint values come first (indices 0-5)
        for (int j = 0; j < 6; j++) {
            command[j] = path[i]->q[j];
        }
        
        // Calculate speed and acceleration for nodes after the first
        double speed = 0.5; // Default - using double instead of float
        double acc = 0.3;   // Default - using double instead of float
        
        if (i > 0) {
            // Calculate distance and velocity
            double joint_dist = TreeManager::distance(path[i-1], path[i]);
            
            // Determine speed based on joint velocity limits
            // Using doubles everywhere for consistency
            speed = std::min(1.0, std::max(0.1, 
                joint_dist * PathExport::CONTROL_RATE / PathExport::MAX_JOINT_VEL));
                
            // If we have 3 or more points, calculate acceleration from velocity changes
            if (i > 1) {
                double prev_dist = TreeManager::distance(path[i-2], path[i-1]);
                double prev_velocity = prev_dist * PathExport::CONTROL_RATE;
                double curr_velocity = joint_dist * PathExport::CONTROL_RATE;
                double accel = std::abs(curr_velocity - prev_velocity) * PathExport::CONTROL_RATE;
                
                // Scale acceleration to 0.1-1.0 range
                acc = std::min(1.0, std::max(0.1, 
                    accel / (PathExport::MAX_JOINT_VEL * 2.0)));
            }
        }
        
        // Speed and acceleration are at the end (indices 6-7)
        command[6] = speed;
        command[7] = acc;
        
        robot_commands.push_back(command);
    }
}

// bool exportPathToYaml(
//     const std::vector<std::shared_ptr<Node>>& path,
//     const std::string& filepath) {
    
//     try {
//         // Create a vector of robot commands first
//         std::vector<std::array<double, 8>> robot_commands;
//         exportPathForRobot(path, robot_commands);
        
//         // Create YAML node
//         YAML::Node root;
//         root["path_metadata"]["version"] = 1.0;
//         root["path_metadata"]["num_waypoints"] = robot_commands.size();
        
//         // Add waypoints
//         YAML::Node waypoints = root["waypoints"];
//         for (size_t i = 0; i < robot_commands.size(); i++) {
//             YAML::Node waypoint;
            
//             // Add joint values
//             for (int j = 0; j < 6; j++) {
//                 waypoint["joints"][j] = robot_commands[i][j];
//             }
            
//             // Add speed and acceleration
//             waypoint["speed"] = robot_commands[i][6];
//             waypoint["acceleration"] = robot_commands[i][7];
            
//             // Add to waypoints sequence
//             waypoints.push_back(waypoint);
//         }
        
//         // Write to file
//         std::ofstream fout(filepath);
//         if (!fout.is_open()) {
//             std::cerr << "Failed to open file for writing: " << filepath << std::endl;
//             return false;
//         }
        
//         fout << root;
//         fout.close();
        
//         std::cout << "Successfully exported path to: " << filepath << std::endl;
//         return true;
//     }
//     catch (const std::exception& e) {
//         std::cerr << "Error exporting path to YAML: " << e.what() << std::endl;
//         return false;
//     }
// }

bool exportPathToYaml(
    const std::vector<std::shared_ptr<Node>>& path,
    const std::string& filepath) {
    
    try {
        // Remove any existing file first
        // If the file exists, remove() will delete it; if not, it simply does nothing.
        if (std::remove(filepath.c_str()) == 0) {
            std::cout << "Previous file " << filepath << " removed." << std::endl;
        }
        // Create a vector of robot commands first
        std::vector<std::array<double, 8>> robot_commands;
        exportPathForRobot(path, robot_commands);
        
        // Create YAML node
        YAML::Node root;
        root["path_metadata"]["version"] = 1.0;
        root["path_metadata"]["num_waypoints"] = robot_commands.size();
        
        // Add waypoints
        YAML::Node waypoints = root["waypoints"];
        for (size_t i = 0; i < robot_commands.size(); i++) {
            YAML::Node waypoint;
            
            // Add joint values
            for (int j = 0; j < 6; j++) {
                waypoint["joints"][j] = robot_commands[i][j];
            }
            
            // Add speed and acceleration
            waypoint["speed"] = robot_commands[i][6];
            waypoint["acceleration"] = robot_commands[i][7];
            
            // Add to waypoints sequence
            waypoints.push_back(waypoint);
        }
        
        // Write to file (this will create a new file)
        std::ofstream fout(filepath);
        if (!fout.is_open()) {
            std::cerr << "Failed to open file for writing: " << filepath << std::endl;
            return false;
        }
        
        fout << root;
        fout.close();
        
        std::cout << "Successfully exported path to: " << filepath << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error exporting path to YAML: " << e.what() << std::endl;
        return false;
    }
}


bool loadPathFromYaml(
    const std::string& filepath,
    std::vector<std::array<double, 8>>& robot_commands) {
    
    try {
        // Clear output vector
        robot_commands.clear();
        
        // Load YAML file
        YAML::Node root = YAML::LoadFile(filepath);
        
        // Check file structure
        if (!root["waypoints"] || !root["waypoints"].IsSequence()) {
            std::cerr << "Invalid YAML file structure: missing or invalid 'waypoints' section" << std::endl;
            return false;
        }
        
        // Process each waypoint
        for (const auto& wp : root["waypoints"]) {
            std::array<double, 8> command;
            
            // Get joint values
            if (!wp["joints"] || !wp["joints"].IsSequence() || wp["joints"].size() != 6) {
                std::cerr << "Invalid waypoint: missing or invalid 'joints' section" << std::endl;
                return false;
            }
            
            for (int j = 0; j < 6; j++) {
                command[j] = wp["joints"][j].as<double>();
            }
            
            // Get speed and acceleration
            if (!wp["speed"] || !wp["acceleration"]) {
                std::cerr << "Invalid waypoint: missing 'speed' or 'acceleration'" << std::endl;
                return false;
            }
            
            command[6] = wp["speed"].as<double>();
            command[7] = wp["acceleration"].as<double>();
            
            // Add to output vector
            robot_commands.push_back(command);
        }
        
        std::cout << "Successfully loaded " << robot_commands.size() 
                  << " waypoints from: " << filepath << std::endl;
        return true;
    }
    catch (const std::exception& e) {
        std::cerr << "Error loading path from YAML: " << e.what() << std::endl;
        return false;
    }
}