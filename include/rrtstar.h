#ifndef RRTSTAR_H
#define RRTSTAR_H

#include <vector>
#include <array>
#include <memory>
#include <random>
#include <cmath>
#include "nanoflann.hpp"
#include <fstream>
#include <map>
#include <string>
#include "inverse_kinematics.h"

struct DHParameters {
    double a;      // link length
    double alpha;  // link twist
    double d;      // link offset
    double theta;  // joint angle
    
    DHParameters(double a_, double alpha_, double d_, double theta_)
        : a(a_), alpha(alpha_), d(d_), theta(theta_) {}
};

class RobotKinematics {
public:
    static std::array<DHParameters, 6> dh_params;
    
    static std::array<double, 16> transformDH(const DHParameters& dh) {
        double ct = std::cos(dh.theta);
        double st = std::sin(dh.theta);
        double ca = std::cos(dh.alpha);
        double sa = std::sin(dh.alpha);
        
        return {
            ct, -st * ca,  st * sa, dh.a * ct,
            st,  ct * ca, -ct * sa, dh.a * st,
            0,   sa,       ca,      dh.d,
            0,   0,        0,       1
        };
    }
    
    static std::array<double, 16> multiplyTransforms(
        const std::array<double, 16>& A, 
        const std::array<double, 16>& B) {
        std::array<double, 16> C;
        for(int i = 0; i < 4; i++) {
            for(int j = 0; j < 4; j++) {
                C[i*4 + j] = 0;
                for(int k = 0; k < 4; k++) {
                    C[i*4 + j] += A[i*4 + k] * B[k*4 + j];
                }
            }
        }
        return C;
    }
    
    static std::array<double, 3> computeFK(const std::array<double, 6>& joints) {
        auto T = transformDH(DHParameters(
            dh_params[0].a, 
            dh_params[0].alpha, 
            dh_params[0].d, 
            joints[0]
        ));
        for (int i = 1; i < 6; i++) {
            auto Ti = transformDH(DHParameters(
                dh_params[i].a, 
                dh_params[i].alpha, 
                dh_params[i].d, 
                joints[i]
            ));
            T = multiplyTransforms(T, Ti);
        }
        return {T[3], T[7], T[11]};
    }
    
    static bool isJointLimitValid(const std::array<double, 6>& joints) {
        const std::array<double, 6> limits_min = {-M_PI, -M_PI/2, -M_PI, -M_PI, -M_PI/2, -M_PI};
        const std::array<double, 6> limits_max = {M_PI, M_PI/2, M_PI, M_PI, M_PI/2, M_PI};
        
        for(size_t i = 0; i < 6; i++) {
            if(joints[i] < limits_min[i] || joints[i] > limits_max[i]) {
                return false;
            }
        }
        return true;
    }
};

struct Node {
    std::array<double, 6> q;  // Joint angles
    double x, y, z;          // End-effector position
    Node* parent = nullptr;
    double cost = 0.0;

    Node(const std::array<double, 6>& q_) : q(q_) {
        auto pos = RobotKinematics::computeFK(q_);
        x = pos[0];
        y = pos[1];
        z = pos[2];
    }

    std::array<double, 3> end_effector_position() const {
        return RobotKinematics::computeFK(q);
    }
};

struct Obstacle {
    std::array<double, 3> min_point;
    std::array<double, 3> max_point;
    Obstacle(const std::array<double, 3>& min, const std::array<double, 3>& max)
        : min_point(min), max_point(max) {}
};

class RRTStar 
{
private:

    InverseKinematics ik_solver;

    // Joint limits for each DOF
    std::array<double, 6> joint_limits_min = {
        -M_PI,    // Base joint (wider range)
        -M_PI/2,  // Shoulder
        -M_PI,    // Elbow
        -M_PI,    // Wrist 1
        -M_PI/2,  // Wrist 2
        -M_PI     // Wrist 3
    };
    std::array<double, 6> joint_limits_max = {
        M_PI,     // Base joint
        M_PI/2,   // Shoulder
        M_PI,     // Elbow
        M_PI,     // Wrist 1
        M_PI/2,   // Wrist 2
        M_PI      // Wrist 3
    };
    // Map parameters
    double map_width, map_height, map_depth;
    double map_min_x, map_min_y, map_min_z; // Add minimum bounds
    double step_size, neighbor_radius, safety_margin;
    int max_iter;

    // Random number generation
    std::mt19937 gen;
    std::uniform_real_distribution<> dis_x, dis_y, dis_z;

    std::array<double, 6> goal_config;

    // Nodes and storage
    std::vector<Node*> nodes;
    std::vector<std::unique_ptr<Node>> node_storage;

    // KD-tree for efficient nearest neighbor search
    struct NodeAdapter {
        std::vector<Node*>& nodes;
        NodeAdapter(std::vector<Node*>& nodes) : nodes(nodes) {}
        inline size_t kdtree_get_point_count() const { return nodes.size(); }
        inline double kdtree_get_pt(const size_t idx, const size_t dim) const {
            return nodes[idx]->q[dim];
        }
        template <typename BBOX>
        bool kdtree_get_bbox(BBOX&) const { return false; }
    };
    NodeAdapter node_adapter;
    std::unique_ptr<nanoflann::KDTreeSingleIndexAdaptor<
        nanoflann::L2_Simple_Adaptor<double, NodeAdapter>,
        NodeAdapter, 6>> kdtree;

    // Start and goal nodes
    std::unique_ptr<Node> start_node;
    std::unique_ptr<Node> goal_node;

    // Static obstacle list
    static std::vector<Obstacle> obstacles;

    // Helper functions
    std::unique_ptr<Node> getRandomNode();
    std::unique_ptr<Node> steer(Node* nearest_node, Node* random_node);
    bool isCollisionFree(Node* node1, Node* node2);
    void rewire(const std::vector<Node*>& neighbors, Node* new_node);
    bool isObstacle(double x, double y, double z);
    bool lineAABBIntersection(const std::array<double, 3>& start,
                              const std::array<double, 3>& end,
                              const std::array<double, 3>& box_min,
                              const std::array<double, 3>& box_max);
    // void plotCuboid(const std::array<double, 3>& min_point,
    //                 const std::array<double, 3>& max_point,
    //                 const std::map<std::string, std::string>& style);

    // Helper functions
    static double normalizeAngle(double angle);
    static double deg2rad(double deg);

public:
    RRTStar(const std::array<double, 6>& start_q, const std::array<double, 6>& goal_q,
            double map_width, double map_height, double map_depth,
            double step_size, double neighbor_radius,
            double safety_margin, int max_iter,
            double min_x = -500, double min_y = -500, double min_z = 0); // Add defaults
    ~RRTStar();
    std::vector<Node*> findPath();
    void getFinalPath(Node* goal_node, std::vector<Node*>& path);
    void optimizePath(std::vector<Node*>& path);
    bool isStateValid(const std::array<double, 6>& q);
    void visualizePath(const std::vector<Node*>& path);
    Node* nearest(Node* target);
    std::vector<Node*> radiusSearch(Node* target, double radius);
    double distance(Node* node1, Node* node2);
    // void smoothPath(std::vector<Node*>& path);

    // Getter for obstacles
    static const std::vector<Obstacle>& getObstacles() {
        return obstacles;
    }
    std::array<double, 6> cartesianToJointSpace(
        double x, double y, double z, double rx, double ry, double rz);

    static void savePlanningResults(const std::vector<Node*>& path);
    static void exportPathData(const std::vector<Node*>& path, const std::string& filename);
};

// void exportData(const std::vector<Node*>& path, const std::vector<Obstacle>& obstacles, const std::string& filename);
// void exportPathData(const std::vector<Node*>& path, const std::string& filename);

#endif // RRTSTAR_H