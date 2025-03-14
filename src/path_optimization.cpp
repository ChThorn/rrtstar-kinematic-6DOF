#include "path_optimization.h"
// #include "robot_kinematics.h" //(umcomment if it doesn't work while compile)
#include <iostream>
#include <algorithm>
#include <cmath>
#include <Eigen/Dense>

PathOptimization::PathOptimization(CollisionDetection& collision_detector, double step_size)
    : collision_detector(collision_detector), step_size(step_size) {
}

void PathOptimization::optimizePath(std::vector<std::shared_ptr<Node>>& path) {
    if(path.size() < 3) return;

    // Phase 1: Shortcutting
    bool changed;
    do {
        changed = false;
        for(size_t i = 0; i < path.size() - 2; ++i) {
            double dist = TreeManager::distance(path[i], path[i+2]);
            if(dist <= step_size * 1.2) {
                auto [free, steps] = isCollisionFree(path[i], path[i+2]);
                if(free) {
                    path.erase(path.begin() + i + 1);
                    changed = true;
                    --i;
                }
            }
        }
    } while(changed && path.size() > 3);

    // Phase 2: Velocity-constrained interpolation
    std::vector<std::shared_ptr<Node>> new_path;
    for(size_t i = 1; i < path.size(); ++i) {
        auto prev_node = path[i-1];
        auto curr_node = path[i];
        
        new_path.push_back(prev_node);
        
        // Joint space distance calculation
        double joint_dist = TreeManager::distance(prev_node, curr_node);
        double dt = joint_dist / MAX_JOINT_VEL;
        int num_steps = std::max(1, static_cast<int>(std::ceil(dt * CONTROL_RATE)));
        
        // Joint space interpolation
        for(int j = 1; j < num_steps; ++j) {
            double t = static_cast<double>(j)/num_steps;
            std::array<double, 6> q;
            
            for(int k = 0; k < 6; ++k) {
                double diff = curr_node->q[k] - prev_node->q[k];
                // Handle angular wrapping for rotational joints
                if(k >= 3) {
                    while(diff > M_PI) diff -= 2*M_PI;
                    while(diff < -M_PI) diff += 2*M_PI;
                }
                q[k] = prev_node->q[k] + t * diff;
                q[k] = RobotKinematics::clampToJointLimits(k, q[k]);
            }
            
            // Create new node with proper parent relationship
            auto new_node = std::make_shared<Node>(q);
            new_node->parent = prev_node; // Parent still uses raw pointer
            new_path.push_back(new_node);
        }
    }
    new_path.push_back(path.back());

    // No need for manual cleanup
    path = std::move(new_path);
}

void PathOptimization::refinePathDynamically(std::vector<std::shared_ptr<Node>>& path) {
    if (path.size() < 3) return;

    // Example: Use cubic splines for smoothing
    std::vector<std::array<double, 3>> smoothed_path;
    for (size_t i = 0; i < path.size(); ++i) {
        auto T = RobotKinematics::computeFK(path[i]->q);
        Eigen::Vector3d pos = T.translation();
        smoothed_path.push_back({pos.x(), pos.y(), pos.z()});
    }

    // Apply cubic spline interpolation
    applyCubicSpline(smoothed_path);

    // Update the path with smoothed configurations using inverseKinematics
    for (size_t i = 0; i < smoothed_path.size(); ++i) {
        // Create Eigen Vector3d from position array
        Eigen::Vector3d target_pos(smoothed_path[i][0], smoothed_path[i][1], smoothed_path[i][2]);
        
        // Use initial guess from previous configuration or the original path
        std::array<double, 6> initial_guess = 
            (i > 0) ? path[i-1]->q : 
            (i < path.size()) ? path[i]->q : 
            path[0]->q;
            
        // Compute inverse kinematics
        std::array<double, 6> new_q = RobotKinematics::inverseKinematics(
            target_pos, 
            initial_guess, 
            1e-3,  // tolerance
            100    // max iterations
        );
        
        // Update path configuration
        path[i]->q = new_q;
    }
}

void PathOptimization::refinePathWithIK(std::vector<std::shared_ptr<Node>>& path) {
    // Track position and orientation for each waypoint
    std::vector<Eigen::Isometry3d> cart_path;
    for(const auto& node : path) {
        auto T = RobotKinematics::computeFK(node->q);
        cart_path.push_back(T);
    }
    
    // Apply cubic spline on position only (could extend to include orientation)
    std::vector<std::array<double, 3>> positions;
    for(const auto& T : cart_path) {
        Eigen::Vector3d pos = T.translation();
        positions.push_back({pos.x(), pos.y(), pos.z()});
    }
    
    applyCubicSpline(positions);
    
    // Create interpolated poses using smoothed positions and original orientations
    std::vector<Eigen::Isometry3d> smoothed_poses;
    int original_size = cart_path.size();
    int smoothed_size = positions.size();
    
    for(size_t i = 0; i < positions.size(); ++i) {
        // Find closest original pose to interpolate orientation
        double t = static_cast<double>(i) / (smoothed_size - 1);
        int idx = std::min(original_size - 1, static_cast<int>(t * (original_size - 1)));
        
        Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
        pose.translation() = Eigen::Vector3d(positions[i][0], positions[i][1], positions[i][2]);
        pose.linear() = cart_path[idx].linear(); // Use orientation from closest original point
        
        smoothed_poses.push_back(pose);
    }
    
    // Generate new path using IK with position and orientation
    std::vector<std::shared_ptr<Node>> new_path;
    for(const auto& pose : smoothed_poses) {
        auto q = RobotKinematics::inverseKinematicsWithOrientation(
            pose, 
            new_path.empty() ? path[0]->q : new_path.back()->q
        );
        new_path.push_back(std::make_shared<Node>(q));
    }
    
    // Replace old path
    path = std::move(new_path);
}

void PathOptimization::applyQuinticSplineWithConstraints(std::vector<std::shared_ptr<Node>>& path) {
    if(path.size() < 4) return;

    const int n = path.size();
    Eigen::MatrixXd q(6, n);
    Eigen::VectorXd time(n);
    
    // Populate joint angles and timestamps
    for(int i = 0; i < n; ++i) {
        for(int j = 0; j < 6; ++j) {
            q(j, i) = path[i]->q[j];
        }
        time(i) = i;
    }
    
    // Compute time scaling to respect velocity limits
    for (int i = 1; i < n; ++i) {
        Eigen::VectorXd dq = q.col(i) - q.col(i-1);
        
        // Find the joint that takes the longest time due to velocity limit
        double max_time_vel = 0.0;
        for (int j = 0; j < 6; ++j) {
            double diff = dq(j);
            if (j >= 3) {
                // Normalize angles for rotational joints
                while(diff > M_PI) diff -= 2*M_PI;
                while(diff < -M_PI) diff += 2*M_PI;
            }
            double joint_time = std::abs(diff) / MAX_JOINT_VEL;
            max_time_vel = std::max(max_time_vel, joint_time);
        }
        
        // Update timestamp to respect velocity constraint (minimum of 0.1s between waypoints)
        time(i) = time(i-1) + std::max(0.1, max_time_vel);
    }

    // Quintic spline interpolation with velocity and acceleration constraints
    std::vector<std::shared_ptr<Node>> smoothed_path;
    const double dt = 1.0 / CONTROL_RATE; // Control rate in seconds
    
    for(double t = time(0); t < time(n-1); t += dt) {
        // Find the segment this time belongs to
        int idx = std::upper_bound(time.data(), time.data()+n, t) - time.data();
        if (idx <= 1) idx = 1;
        if (idx >= n) idx = n-1;
        
        // Local time parameter [0,1] within the segment
        double local_t = (t - time(idx-1)) / (time(idx) - time(idx-1));
        local_t = std::max(0.0, std::min(1.0, local_t)); // Clamp to [0,1]
        
        // Quintic polynomial basis
        double t2 = local_t * local_t;
        double t3 = t2 * local_t;
        double t4 = t3 * local_t;
        double t5 = t4 * local_t;
        
        std::array<double, 6> q_new;
        std::array<double, 6> dq_new; // Velocity (for constraint checking)
        
        for(int j = 0; j < 6; ++j) {
            double q0 = q(j, idx-1);
            double q1 = q(j, idx);
            
            // Boundary conditions for smooth velocity and acceleration
            double dq0 = (idx > 1) ? (q(j, idx) - q(j, idx-2)) / (time(idx) - time(idx-2)) : 0.0;
            double dq1 = (idx < n-1) ? (q(j, idx+1) - q(j, idx-1)) / (time(idx+1) - time(idx-1)) : 0.0;
            double ddq0 = 0.0; // Zero acceleration at endpoints for smoothness
            double ddq1 = 0.0;
            
            // Limit velocities to MAX_JOINT_VEL
            dq0 = std::min(MAX_JOINT_VEL, std::max(-MAX_JOINT_VEL, dq0));
            dq1 = std::min(MAX_JOINT_VEL, std::max(-MAX_JOINT_VEL, dq1));
            
            // Compute quintic polynomial coefficients
            double a0 = q0;
            double a1 = dq0;
            double a2 = 0.5 * ddq0;
            double a3 = 10.0 * (q1 - q0) - 6.0 * dq0 - 4.0 * dq1 - 1.5 * ddq0 + 0.5 * ddq1;
            double a4 = 15.0 * (q0 - q1) + 8.0 * dq0 + 7.0 * dq1 + 1.5 * ddq0 - ddq1;
            double a5 = 6.0 * (q1 - q0) - 3.0 * (dq0 + dq1) - 0.5 * (ddq0 - ddq1);
            
            // Evaluate position and velocity
            q_new[j] = a0 + a1*local_t + a2*t2 + a3*t3 + a4*t4 + a5*t5;
            dq_new[j] = a1 + 2.0*a2*local_t + 3.0*a3*t2 + 4.0*a4*t3 + 5.0*a5*t4;
            
            // Velocity constraint enforcement
            double vel_scale = 1.0;
            if (std::abs(dq_new[j]) > MAX_JOINT_VEL) {
                vel_scale = std::min(vel_scale, MAX_JOINT_VEL / std::abs(dq_new[j]));
            }
            
            // Apply velocity scaling if needed
            if (vel_scale < 1.0) {
                // Adjust position based on scaled velocity
                // This is a simplified approach; a better method would replan the entire trajectory
                dq_new[j] *= vel_scale;
                q_new[j] = q0 + dq_new[j] * ((time(idx) - time(idx-1)) * local_t);
            }
        }
        
        auto node = std::make_shared<Node>(q_new);
        smoothed_path.push_back(node);
    }

    // Add the final node if not already there
    if (!smoothed_path.empty()) {
        double dist_to_goal = TreeManager::distance(smoothed_path.back(), path.back());
        if (dist_to_goal > 1e-6) {
            smoothed_path.push_back(std::make_shared<Node>(path.back()->q));
        }
    }

    // Cleanup and replace path
    path = std::move(smoothed_path);
}

void PathOptimization::applyCubicSpline(const std::vector<std::array<double, 3>>& points) {
    size_t n = points.size();
    if (n < 2) return; // Need at least two points for interpolation

    // Step 1: Extract x, y, z coordinates from the points
    Eigen::VectorXd x(n), y(n), z(n);
    for (size_t i = 0; i < n; ++i) {
        x(i) = points[i][0];
        y(i) = points[i][1];
        z(i) = points[i][2];
    }

    // Step 2: Compute distances between consecutive points (t values)
    Eigen::VectorXd t(n);
    t(0) = 0.0;
    for (size_t i = 1; i < n; ++i) {
        double dx = points[i][0] - points[i - 1][0];
        double dy = points[i][1] - points[i - 1][1];
        double dz = points[i][2] - points[i - 1][2];
        t(i) = t(i - 1) + std::sqrt(dx * dx + dy * dy + dz * dz);
    }

    // Step 3: Construct the tridiagonal system for second derivatives
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(n, n);
    Eigen::VectorXd b_x = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd b_y = Eigen::VectorXd::Zero(n);
    Eigen::VectorXd b_z = Eigen::VectorXd::Zero(n);

    // Natural spline boundary conditions
    A(0, 0) = 1.0;
    A(n - 1, n - 1) = 1.0;

    for (size_t i = 1; i < n - 1; ++i) {
        double h_i = t(i) - t(i - 1);
        double h_ip1 = t(i + 1) - t(i);

        A(i, i - 1) = h_i;
        A(i, i) = 2.0 * (h_i + h_ip1);
        A(i, i + 1) = h_ip1;

        b_x(i) = 3.0 * ((x(i + 1) - x(i)) / h_ip1 - (x(i) - x(i - 1)) / h_i);
        b_y(i) = 3.0 * ((y(i + 1) - y(i)) / h_ip1 - (y(i) - y(i - 1)) / h_i);
        b_z(i) = 3.0 * ((z(i + 1) - z(i)) / h_ip1 - (z(i) - z(i - 1)) / h_i);
    }

    // Step 4: Solve for second derivatives
    Eigen::VectorXd k_x = A.colPivHouseholderQr().solve(b_x);
    Eigen::VectorXd k_y = A.colPivHouseholderQr().solve(b_y);
    Eigen::VectorXd k_z = A.colPivHouseholderQr().solve(b_z);

    // Step 5: Generate smoothed points using cubic polynomials
    std::vector<std::array<double, 3>> smoothed_points;
    const int num_samples = 10; // Number of samples per segment

    for (size_t i =.0; i < n - 1; ++i) {
        double h = t(i + 1) - t(i);
        for (int j = 0; j <= num_samples; ++j) {
            double u = static_cast<double>(j) / num_samples;
            double a = 2.0 * u * u * u - 3.0 * u * u + 1.0;
            double b = -2.0 * u * u * u + 3.0 * u * u;
            double c = u * u * u - 2.0 * u * u + u;
            double d = u * u * u - u * u;

            double x_smooth = a * x(i) + b * x(i + 1) + h * (c * k_x(i) + d * k_x(i + 1));
            double y_smooth = a * y(i) + b * y(i + 1) + h * (c * k_y(i) + d * k_y(i + 1));
            double z_smooth = a * z(i) + b * z(i + 1) + h * (c * k_z(i) + d * k_z(i + 1));

            smoothed_points.push_back({x_smooth, y_smooth, z_smooth});
        }
    }

    // Output information
    std::cout << "Smoothed path generated with " << smoothed_points.size() << " points.\n";
}

PathQualityMetrics PathOptimization::evaluatePathQuality(const std::vector<std::shared_ptr<Node>>& path) {
    PathQualityMetrics metrics;
    if (path.size() < 2) return metrics;
    
    std::vector<double> step_sizes;
    for (size_t i = 1; i < path.size(); ++i) {
        double step = TreeManager::distance(path[i-1], path[i]);
        metrics.total_length += step;
        metrics.max_step = std::max(metrics.max_step, step);
        step_sizes.push_back(step);
    }
    
    metrics.avg_step = metrics.total_length / (path.size() - 1);
    
    // Calculate path smoothness as variance in step sizes
    double variance = 0.0;
    for (double step : step_sizes) {
        variance += std::pow(step - metrics.avg_step, 2);
    }
    metrics.smoothness = step_sizes.empty() ? 0.0 : std::sqrt(variance / step_sizes.size());
    
    return metrics;
}

// void PathOptimization::exportPathForRobot(
//     const std::vector<std::shared_ptr<Node>>& path,
//     std::vector<std::array<double, 8>>& robot_commands) {
    
//     robot_commands.clear();
    
//     // Handle empty path
//     if (path.empty()) return;
    
//     // Process each waypoint
//     for (size_t i = 0; i < path.size(); i++) {
//         std::array<double, 8> command;
        
//         // Joint values come first (indices 0-5)
//         for (int j = 0; j < 6; j++) {
//             command[j] = path[i]->q[j];
//         }
        
//         // Calculate speed and acceleration for nodes after the first
//         double speed = 0.5; // Default - using double instead of float
//         double acc = 0.3;   // Default - using double instead of float
        
//         if (i > 0) {
//             // Calculate distance and velocity
//             double joint_dist = TreeManager::distance(path[i-1], path[i]);
            
//             // Determine speed based on joint velocity limits
//             // Using doubles everywhere for consistency
//             speed = std::min(1.0, std::max(0.1, 
//                 joint_dist * CONTROL_RATE / MAX_JOINT_VEL));
                
//             // If we have 3 or more points, calculate acceleration from velocity changes
//             if (i > 1) {
//                 double prev_dist = TreeManager::distance(path[i-2], path[i-1]);
//                 double prev_velocity = prev_dist * CONTROL_RATE;
//                 double curr_velocity = joint_dist * CONTROL_RATE;
//                 double accel = std::abs(curr_velocity - prev_velocity) * CONTROL_RATE;
                
//                 // Scale acceleration to 0.1-1.0 range
//                 acc = std::min(1.0, std::max(0.1, 
//                     accel / (MAX_JOINT_VEL * 2.0)));
//             }
//         }
        
//         // Speed and acceleration are at the end (indices 6-7)
//         command[6] = speed;
//         command[7] = acc;
        
//         robot_commands.push_back(command);
//     }
// }

std::pair<bool, int> PathOptimization::isCollisionFree(
    std::shared_ptr<Node> node1, std::shared_ptr<Node> node2) {
    
    const double dist = TreeManager::distance(node1, node2);
    const int steps = std::max(20, static_cast<int>(dist / (step_size * 0.1))); // Higher resolution

    // Check every interpolated state at higher resolution
    for (int i = 0; i <= steps; ++i) {
        double t = static_cast<double>(i) / steps;
        std::array<double, 6> q;
        for (int j = 0; j < 6; ++j) {
            q[j] = node1->q[j] + t * (node2->q[j] - node1->q[j]);
        }
        if (!collision_detector.isStateValid(q)) {
            return {false, steps};
        }
    }
    return {true, steps};
}