#include "path_planning.h"
#include <iostream>
#include <cmath>
#include <limits>
#include <algorithm>

PathPlanner::PathPlanner(
    TreeManager& tree_manager,
    CollisionDetection& collision_detector,
    PathOptimization& path_optimizer,
    double step_size,
    double neighbor_radius,
    int max_iter
) : tree_manager(tree_manager),
    collision_detector(collision_detector),
    path_optimizer(path_optimizer),
    step_size(step_size),
    neighbor_radius(neighbor_radius),
    max_iter(max_iter),
    goal_pose(Eigen::Isometry3d::Identity()) {
}

std::vector<std::shared_ptr<Node>> PathPlanner::globalPlanner() {
    bool found_path = false;
    std::shared_ptr<Node> final_node = nullptr;
    double goal_threshold = step_size * 1.2;  // Position threshold
    double orientation_threshold = 0.2;       // About 11 degrees in radians
    int stall_count = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    
    std::cout << "Starting global planning with position threshold: " << goal_threshold << std::endl;
    std::cout << "Orientation threshold: " << orientation_threshold << " radians" << std::endl;
    
    auto start_node = tree_manager.getStartNode();
    auto goal_node = tree_manager.getGoalNode();
    
    std::cout << "Start position is valid: " << collision_detector.isStateValid(start_node->q) << std::endl;
    std::cout << "Goal position is valid: " << collision_detector.isStateValid(goal_node->q) << std::endl;
    
    // Add goal node to tree occasionally
    const int goal_attempt_freq = 20;
    const int stall_limit = 50;

    for (int i = 0; i < max_iter && !found_path; ++i) {
        std::shared_ptr<Node> new_node = nullptr;
        
        // Periodically attempt to connect directly to goal
        if (i % goal_attempt_freq == 0) {
            auto nearest_to_goal = tree_manager.nearest(goal_node);
            auto goal_step = steer(nearest_to_goal, goal_node);
            auto [collision_free, steps] = path_optimizer.isCollisionFree(nearest_to_goal, goal_step);
            if (collision_free) {
                new_node = goal_step;
                
                // Check if we're close enough to the goal in both position and orientation
                double dist_to_goal = TreeManager::distance(new_node, goal_node);
                
                // Check orientation alignment
                auto current_pose = RobotKinematics::computeFK(new_node->q);
                Eigen::Matrix3d current_rotation = current_pose.rotation();
                Eigen::Matrix3d goal_rotation = goal_pose.rotation();
                
                // Compute orientation difference (as angle)
                Eigen::Matrix3d rotation_diff = current_rotation.transpose() * goal_rotation;
                Eigen::AngleAxisd angle_axis(rotation_diff);
                double orientation_diff = angle_axis.angle();
                
                if (dist_to_goal < goal_threshold && orientation_diff < orientation_threshold) {
                    found_path = true;
                    final_node = new_node;
                    std::cout << "Goal reached directly! Distance: " << dist_to_goal 
                              << ", Orientation diff: " << orientation_diff << " rad" << std::endl;
                    break;
                }
            }
        }
        
        // If not connecting to goal or if that failed, get random node
        if (!new_node) {
            auto random_node = tree_manager.getRandomNode(i, max_iter);
            auto nearest_node = tree_manager.nearest(random_node);
            auto step_node = steer(nearest_node, random_node);
            
            auto [collision_free, steps] = path_optimizer.isCollisionFree(nearest_node, step_node);
            if (collision_free) {
                new_node = step_node;
            } else {
                continue;
            }
        }
        
        // Add node to tree
        auto nearest_node = tree_manager.nearest(new_node);
        new_node->parent = nearest_node;
        
        // Calculate cost
        if (auto locked_parent = new_node->parent.lock()) {
            new_node->cost = locked_parent->cost + TreeManager::distance(locked_parent, new_node);
        } else {
            new_node->cost = 0;
        }
        
        tree_manager.addNode(new_node);
        
        // Check progress
        double dist_to_goal = TreeManager::distance(new_node, goal_node);
        if (dist_to_goal < best_distance) {
            best_distance = dist_to_goal;
            stall_count = 0;
            std::cout << "New best distance: " << best_distance << std::endl;
        } else {
            stall_count++;
            if (stall_count > stall_limit) {
                std::cout << "Breaking due to lack of progress after " 
                          << stall_limit << " iterations" << std::endl;
                break;
            }
        }
        
        // Check if we reached the goal (with pose consideration)
        auto current_pose = RobotKinematics::computeFK(new_node->q);
        Eigen::Matrix3d current_rotation = current_pose.rotation();
        Eigen::Matrix3d goal_rotation = goal_pose.rotation();
        Eigen::Matrix3d rotation_diff = current_rotation.transpose() * goal_rotation;
        Eigen::AngleAxisd angle_axis(rotation_diff);
        double orientation_diff = angle_axis.angle();
        
        if (dist_to_goal < goal_threshold && orientation_diff < orientation_threshold) {
            found_path = true;
            final_node = new_node;
            std::cout << "Goal reached! Distance: " << dist_to_goal 
                      << ", Orientation diff: " << orientation_diff << " rad" << std::endl;
            break;
        }
        
        // Periodic progress report
        if (i % 100 == 0) {
            std::cout << "Iteration " << i << ", Best distance: " << best_distance << std::endl;
        }
        
        // Rewire nearby nodes
        auto neighbors = tree_manager.radiusSearch(new_node, neighbor_radius);
        rewire(neighbors, new_node);
    }
    
    std::vector<std::shared_ptr<Node>> path;
    if (found_path && final_node) {
        tree_manager.getFinalPath(final_node, path);
        std::cout << "Path found with " << path.size() << " nodes" << std::endl;
    } else {
        std::cout << "No path found! Best distance to goal: " << best_distance << std::endl;
    }
    
    return path;
}

void PathPlanner::localPlanner(std::vector<std::shared_ptr<Node>>& path) {
    // Apply path smoothing or optimization
    path_optimizer.optimizePath(path);

    // Apply additional local refinements
    path_optimizer.refinePathDynamically(path);
}

std::shared_ptr<Node> PathPlanner::steer(
    std::shared_ptr<Node> from_node, 
    std::shared_ptr<Node> to_node
) {
    std::array<double, 6> new_q;
    std::array<double, 6> dq;
    double total_dist = 0.0;

    // Calculate direction with angle wrapping
    for(size_t i = 0; i < 6; ++i) {
        double raw_diff = to_node->q[i] - from_node->q[i];
        dq[i] = (i >= 3) ? RobotKinematics::wrapAngle(raw_diff) : raw_diff;
        total_dist += dq[i] * dq[i];
    }
    
    total_dist = std::sqrt(total_dist);
    if(total_dist < 1e-6) return from_node;

    // Adaptive step calculation
    double adaptive_step = step_size * (1.0 + 0.5 * std::min(1.0, 
        TreeManager::distance(from_node, tree_manager.getGoalNode()) / neighbor_radius));
    double scale = std::min(1.0, adaptive_step / total_dist);

    // Apply movement with joint limits
    for(size_t i = 0; i < 6; ++i) {
        new_q[i] = RobotKinematics::clampToJointLimits(i, from_node->q[i] + dq[i] * scale);
    }

    return std::make_shared<Node>(new_q);
}

void PathPlanner::rewire(const std::vector<std::shared_ptr<Node>>& neighbors, std::shared_ptr<Node> new_node) {
    for(auto neighbor : neighbors) {
        auto [collision_free, steps] = path_optimizer.isCollisionFree(neighbor, new_node);
        if(collision_free) {
            // Base cost = distance
            double dist_cost = TreeManager::distance(neighbor, new_node);
            
            // Angular change penalty
            double angular_cost = 0;
            for(int i = 0; i < 6; ++i) {
                double diff = new_node->q[i] - neighbor->q[i];
                if(i >= 3) {
                    diff = RobotKinematics::wrapAngle(diff);
                }
                angular_cost += std::abs(diff);
            }
            
            // Orientation alignment with goal
            double orientation_cost = 0.0;
            auto current_pose = RobotKinematics::computeFK(new_node->q);
            Eigen::Matrix3d current_rotation = current_pose.rotation();
            Eigen::Matrix3d goal_rotation = goal_pose.rotation();
            Eigen::Matrix3d rotation_diff = current_rotation.transpose() * goal_rotation;
            Eigen::AngleAxisd angle_axis(rotation_diff);
            orientation_cost = angle_axis.angle(); // Angle in radians
            
            // Angular velocity consistency penalty (jerk minimization)
            double jerk_cost = 0.0;
            if (auto parent = neighbor->parent.lock()) {
                // Parent exists
                std::array<double, 6> prev_velocity, curr_velocity;
                
                // Compute previous angular velocity
                for (int i = 0; i < 6; ++i) {
                    double diff = neighbor->q[i] - parent->q[i];
                    if (i >= 3) {
                        diff = RobotKinematics::wrapAngle(diff);
                    }
                    prev_velocity[i] = diff;
                }
                
                // Compute current angular velocity
                for (int i = 0; i < 6; ++i) {
                    double diff = new_node->q[i] - neighbor->q[i];
                    if (i >= 3) {
                        diff = RobotKinematics::wrapAngle(diff);
                    }
                    curr_velocity[i] = diff;
                }
                
                // Compute change in angular velocity (jerk)
                for (int i = 0; i < 6; ++i) {
                    double velocity_diff = curr_velocity[i] - prev_velocity[i];
                    jerk_cost += velocity_diff * velocity_diff;
                }
                
                jerk_cost = std::sqrt(jerk_cost);
            }
            
            // Combine costs with appropriate weights
            double total_cost = neighbor->cost + 
                                0.4 * dist_cost +        // Distance weight
                                0.2 * angular_cost +     // Angular change weight
                                0.2 * jerk_cost +        // Jerk minimization weight
                                0.2 * orientation_cost;  // Goal orientation alignment weight
            
            if(total_cost < new_node->cost) {
                new_node->parent = neighbor;
                new_node->cost = total_cost;
            }
        }
    }
}

std::vector<std::shared_ptr<Node>> PathPlanner::findPath() {
    // Phase 0: Global planning with retries
    auto path = globalPlanner();
    if (path.empty()) {
        std::cerr << "Initial planning failed. Adjusting parameters and retrying..." << std::endl;
        step_size *= 0.8;
        neighbor_radius *= 1.2;
        path = globalPlanner();
        if (path.empty()) {
            throw std::runtime_error("Path planning failed after retries.");
        }
    }

    // Add trajectory validation before continuing
    for (size_t i = 1; i < path.size(); ++i) {
        auto [collision_free, steps] = path_optimizer.isCollisionFree(path[i-1], path[i]);
        if (!collision_free) {
            std::cerr << "ERROR: Collision detected in final path segment " << i-1 << "->" << i << std::endl;
            return {}; // Fail fast
        }
    }

    std::cout << "Initial path found with " << path.size() << " nodes" << std::endl;
    
    // Save initial path metrics
    auto initial_metrics = path_optimizer.evaluatePathQuality(path);
    std::cout << "Initial path metrics:" << std::endl;
    std::cout << "Total length: " << initial_metrics.total_length << std::endl;
    std::cout << "Max step: " << initial_metrics.max_step << std::endl;
    std::cout << "Average step: " << initial_metrics.avg_step << std::endl;
    std::cout << "Smoothness: " << initial_metrics.smoothness << std::endl;
    
    // Phase 1: Basic path optimization (shortcutting)
    std::cout << "Applying basic path optimization..." << std::endl;
    path_optimizer.optimizePath(path);
    
    // Phase 2: Cartesian smoothing with accurate IK
    if(!path.empty() && path.size() > 2) {
        std::cout << "Refining path with IK..." << std::endl;
        path_optimizer.refinePathWithIK(path);
    }
    
    // Phase 3: Joint-space smoothing with velocity constraints
    if(!path.empty() && path.size() > 3) {
        std::cout << "Applying velocity-constrained quintic spline..." << std::endl;
        path_optimizer.applyQuinticSplineWithConstraints(path);
    }
    
    // Phase 4: Final optimization for velocity consistency
    if(!path.empty() && path.size() > 2) {
        std::cout << "Performing final velocity-based optimization..." << std::endl;
        path_optimizer.optimizePath(path);
    }
    
    // Output final path metrics
    if(!path.empty()) {
        auto final_metrics = path_optimizer.evaluatePathQuality(path);
        std::cout << "\nFinal path metrics:" << std::endl;
        std::cout << "Path nodes: " << path.size() << std::endl;
        std::cout << "Total length: " << final_metrics.total_length << std::endl;
        std::cout << "Max step: " << final_metrics.max_step << std::endl;
        std::cout << "Average step: " << final_metrics.avg_step << std::endl;
        std::cout << "Smoothness: " << final_metrics.smoothness << std::endl;
        
        // Verify all path steps meet velocity constraints
        bool velocity_constraints_met = true;
        for(size_t i = 1; i < path.size(); ++i) {
            double step_size = TreeManager::distance(path[i-1], path[i]);
            if(step_size > path_optimizer.MAX_JOINT_VEL/path_optimizer.CONTROL_RATE * 1.1) {
                std::cout << "Warning: Velocity constraint violated at step " << i 
                          << ": " << step_size << " > " 
                          << path_optimizer.MAX_JOINT_VEL/path_optimizer.CONTROL_RATE << std::endl;
                velocity_constraints_met = false;
            }
        }
        
        if(velocity_constraints_met) {
            std::cout << "All velocity constraints satisfied!" << std::endl;
        }
    }

    return path;
}

void PathPlanner::setGoalPose(const Eigen::Isometry3d& pose) {
    goal_pose = pose;
}

const Eigen::Isometry3d& PathPlanner::getGoalPose() const {
    return goal_pose;
}