#ifndef RRT_PLANNER_H
#define RRT_PLANNER_H

#include <unordered_map>
#include <map>
#include <queue>

#include "Planner.h"


class RRT_Planner : public Planner {
 public:

    RRT_Planner(int K,
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

    int getNearestNeighbourId(const std::vector<double>& config);

    void growTree();
    void extend(const std::vector<double>& config);
    void extendWithInterpolation(const std::vector<double>& config);

    void addVertex(const std::vector<double>& config);
    void addEdge(const int id1, const int id2);

    std::vector<std::vector<double>> findPath();

    int num_samples;
    // Epsilon: the longest distance to extend while growing the tree
    double eps;
    // If within reached_threshold, considered as goal reached
    double reached_threshold;

    bool goal_reached;
    int goal_parent;

    std::vector<int> parent;
    std::vector<std::vector<double>> tree;

};


#endif