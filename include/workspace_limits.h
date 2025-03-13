#ifndef WORKSPACE_LIMITS_H
#define WORKSPACE_LIMITS_H

/**
 * This file defines the workspace limits for the robot.
 * It serves as a central place to set boundaries that will be used by
 * both inverse kinematics calculations and path planning algorithms.
 */

namespace WorkspaceLimits {
    // Cartesian workspace boundaries
    static constexpr double MIN_X = -1500.0; // mm
    static constexpr double MAX_X = 1500.0;  // mm
    static constexpr double MIN_Y = -1500.0; // mm
    static constexpr double MAX_Y = 1500.0;  // mm
    static constexpr double MIN_Z = -500.0;  // mm
    static constexpr double MAX_Z = 2000.0;  // mm
    
    // Derived dimensions
    static constexpr double WIDTH = MAX_X - MIN_X;
    static constexpr double HEIGHT = MAX_Y - MIN_Y;
    static constexpr double DEPTH = MAX_Z - MIN_Z;
    
    // Check if a point is within workspace boundaries
    inline bool isPointInWorkspace(double x, double y, double z) {
        return (x >= MIN_X && x <= MAX_X &&
                y >= MIN_Y && y <= MAX_Y &&
                z >= MIN_Z && z <= MAX_Z);
    }
    
    // Return normalized position within workspace (0-1)
    inline double getNormalizedX(double x) { return (x - MIN_X) / WIDTH; }
    inline double getNormalizedY(double y) { return (y - MIN_Y) / HEIGHT; }
    inline double getNormalizedZ(double z) { return (z - MIN_Z) / DEPTH; }
}

#endif // WORKSPACE_LIMITS_H