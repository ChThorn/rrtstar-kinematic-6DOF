#include <iostream>
#include <vector>
#include <cmath>
#include <limits>
#include <random>
#include <algorithm>  // Added include for std::reverse

struct Node {
    std::vector<double> position;
    Node* parent;
    double cost;
    Node(std::vector<double> pos) : position(pos), parent(nullptr), cost(0.0) {}
};

class RRTStar {
public:
    RRTStar(std::vector<double> start, std::vector<double> goal, std::vector<std::pair<double, double>> joint_limits,
            std::vector<std::pair<std::vector<double>, double>> obstacle_list, double step_size = 0.2, int max_iter = 2000, double search_radius = 2.0, double goal_bias = 0.1)
        : start(start), goal(goal), joint_limits(joint_limits), obstacle_list(obstacle_list), step_size(step_size), max_iter(max_iter), search_radius(search_radius), goal_bias(goal_bias) {
        start_node = new Node(start);
        goal_node = new Node(goal);
        node_list.push_back(start_node);
    }

    ~RRTStar() {
        for (Node* node : node_list) {
            delete node;
        }
    }

    std::vector<Node*> getPath() {
        std::vector<Node*> path;
        Node* node = goal_node;
        while (node->parent != nullptr) {
            path.push_back(node);
            node = node->parent;
        }
        path.push_back(start_node);
        std::reverse(path.begin(), path.end());
        return path;
    }

    void plan() {
        for (int i = 0; i < max_iter; ++i) {
            Node* rand_node = getRandomNode();
            Node* nearest_node = getNearestNode(rand_node);
            Node* new_node = steer(nearest_node, rand_node);
            if (isCollisionFree(new_node)) {
                std::vector<Node*> near_nodes = findNearNodes(new_node);
                new_node->parent = chooseParent(new_node, near_nodes);
                node_list.push_back(new_node);
                rewire(new_node, near_nodes);
                double dist_to_goal = distance(new_node->position, goal_node->position);
                if (dist_to_goal <= step_size) {
                    goal_node->parent = new_node;
                    goal_node->cost = new_node->cost + dist_to_goal;
                    std::cout << "Goal reached!" << std::endl;
                    return;
                }
                if (i % 100 == 0) {
                    std::cout << "Iteration: " << i << ", Nodes: " << node_list.size() << ", Distance to goal: " << dist_to_goal << std::endl;
                }
            }
        }
    }

private:
    Node* start_node;
    Node* goal_node;
    std::vector<Node*> node_list;
    std::vector<double> start;
    std::vector<double> goal;
    std::vector<std::pair<double, double>> joint_limits;
    std::vector<std::pair<std::vector<double>, double>> obstacle_list;
    double step_size;
    int max_iter;
    double search_radius;
    double goal_bias;

    Node* getRandomNode() {
        std::vector<double> rand_pos(joint_limits.size());
        if (randomDouble(0, 1) < goal_bias) {
            rand_pos = goal;
        } else {
            for (size_t i = 0; i < joint_limits.size(); ++i) {
                rand_pos[i] = randomDouble(joint_limits[i].first, joint_limits[i].second);
            }
        }
        return new Node(rand_pos);
    }

    Node* getNearestNode(Node* rand_node) {
        double min_dist = std::numeric_limits<double>::max();
        Node* nearest_node = nullptr;
        for (Node* node : node_list) {
            double dist = distance(node->position, rand_node->position);
            if (dist < min_dist) {
                min_dist = dist;
                nearest_node = node;
            }
        }
        return nearest_node;
    }

    bool isCollisionFree(Node* node) {
        for (const auto& obstacle : obstacle_list) {
            if (distance(node->position, obstacle.first) <= obstacle.second) {
                return false;
            }
        }
        return true;
    }

    Node* steer(Node* from_node, Node* to_node) {
        std::vector<double> direction(to_node->position.size());
        double dist = distance(from_node->position, to_node->position);
        for (size_t i = 0; i < direction.size(); ++i) {
            direction[i] = to_node->position[i] - from_node->position[i];
            direction[i] = (direction[i] / dist) * step_size;
        }
        std::vector<double> new_position(from_node->position.size());
        for (size_t i = 0; i < new_position.size(); ++i) {
            new_position[i] = from_node->position[i] + direction[i];
        }
        Node* new_node = new Node(new_position);
        new_node->parent = from_node;
        new_node->cost = from_node->cost + distance(from_node->position, new_position);
        return new_node;
    }

    Node* chooseParent(Node* new_node, const std::vector<Node*>& near_nodes) {
        if (near_nodes.empty()) {
            return new_node->parent;
        }
        double min_cost = std::numeric_limits<double>::max();
        Node* min_node = nullptr;
        for (Node* near_node : near_nodes) {
            double cost = near_node->cost + distance(near_node->position, new_node->position);
            if (cost < min_cost && isCollisionFree(new_node)) {
                min_cost = cost;
                min_node = near_node;
            }
        }
        new_node->cost = min_cost;
        return min_node;
    }

    void rewire(Node* new_node, const std::vector<Node*>& near_nodes) {
        for (Node* near_node : near_nodes) {
            double new_cost = new_node->cost + distance(new_node->position, near_node->position);
            if (new_cost < near_node->cost && isCollisionFree(near_node)) {
                near_node->parent = new_node;
                near_node->cost = new_cost;
            }
        }
    }

    std::vector<Node*> findNearNodes(Node* new_node) {
        int n = node_list.size();
        double r = search_radius * sqrt((log(n) / n));
        std::vector<Node*> near_nodes;
        for (Node* node : node_list) {
            if (distance(node->position, new_node->position) <= r) {
                near_nodes.push_back(node);
            }
        }
        return near_nodes;
    }

    double distance(const std::vector<double>& a, const std::vector<double>& b) {
        double dist = 0.0;
        for (size_t i = 0; i < a.size(); ++i) {
            dist += pow(a[i] - b[i], 2);
        }
        return sqrt(dist);
    }

    double randomDouble(double min, double max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<> dis(min, max);
        return dis(gen);
    }
};

int main() {
    std::vector<std::pair<double, double>> joint_limits = {
        {-M_PI, M_PI}, {-M_PI, M_PI}, {-M_PI, M_PI},
        {-M_PI, M_PI}, {-M_PI, M_PI}, {-M_PI, M_PI}
    };
    std::vector<std::pair<std::vector<double>, double>> obstacle_list = {
        {{1.0, 1.0, 1.0, 1.0, 1.0, 1.0}, 0.5}
    };
    std::vector<double> start = {0, 0, 0, 0, 0, 0};
    std::vector<double> goal = {M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2, M_PI/2};

    RRTStar rrt_star(start, goal, joint_limits, obstacle_list);
    rrt_star.plan();

    std::vector<Node*> path = rrt_star.getPath();
    if (path.empty()) {
        std::cout << "No path found" << std::endl;
    } else {
        std::cout << "Path found" << std::endl;
        for (Node* node : path) {
            for (double pos : node->position) {
                std::cout << pos << " ";
            }
            std::cout << std::endl;
        }
    }

    return 0;
}