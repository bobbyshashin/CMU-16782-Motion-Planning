#ifndef RRT_CONNECT_PLANNER_H
#define RRT_CONNECT_PLANNER_H

#include <unordered_map>
#include <map>
#include <queue>

#include "Planner.h"


class RRT_Connect_Planner : public Planner {
 public:

    RRT_Connect_Planner(int K,
        double eps,
        double reached_threshold,
        int DOF, 
        double* joint_limits,
        double* start,
        double* goal,
        int map_size_x, 
        int map_size_y, 
        double* map, 
        int link_length_cells, 
        double pi,
        double goal_bias_weight) : Planner(DOF, joint_limits, start, goal, map_size_x, map_size_y, map, link_length_cells, pi, goal_bias_weight), num_samples(K), eps(eps), reached_threshold(reached_threshold) {}
    void init();

    int getNearestNeighbourId(const std::vector<double>& config, int tree_id);

    void growTree();
    bool extend(const std::vector<double>& config);
    bool connect(const std::vector<double>& config);

    void addVertex(const std::vector<double>& config, const int tree_id);
    void addEdge(const int id1, const int id2, const int tree_id);

    void swapTree();
    std::vector<std::vector<double>> interpolatePath(std::vector<std::vector<double>>& path);
    std::vector<std::vector<double>> findPath();

    int num_samples;
    // Epsilon: the longest distance to extend while growing the tree
    double eps;
    // If within reached_threshold, considered as goal reached
    double reached_threshold;

    bool goal_reached;

    int curr_tree_id;
    int connection_id1;
    int connection_id2;

    std::unordered_map<int, int> parent1;
    std::unordered_map<int, int> parent2;
    std::unordered_map<int, std::vector<double>> tree1;
    std::unordered_map<int, std::vector<double>> tree2;

};

#endif