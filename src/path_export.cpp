#include "path_export.h"
#include "tree_management.h"  // For Node definition and distance function
#include <algorithm>  // For std::min, std::max
#include <cmath>      // For std::abs

void exportPathForRobot(
    const std::vector<std::shared_ptr<Node>>& path,
    std::vector<std::array<double, 8>>& robot_commands) {
    
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