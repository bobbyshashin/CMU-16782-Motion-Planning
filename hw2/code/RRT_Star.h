#ifndef RRT_STAR_PLANNER_H
#define RRT_STAR_PLANNER_H

#include <unordered_map>
#include <map>
#include <queue>

#include "Planner.h"


class RRT_Star_Planner : public Planner {
 public:

    RRT_Star_Planner(int K,
        double eps,
        double reached_threshold,
        double neighbour_radius,
        int DOF, 
        double* joint_limits,
        double* start,
        double* goal,
        int map_size_x, 
        int map_size_y, 
        double* map, 
        int link_length_cells, 
        double pi,
        double goal_bias_weight) : Planner(DOF, joint_limits, start, goal, map_size_x, map_size_y, map, link_length_cells, pi, goal_bias_weight), num_samples(K), eps(eps), reached_threshold(reached_threshold), neighbour_radius(neighbour_radius) {}
    void init();

    int getNearestNeighbourId(const std::vector<double>& config);
    std::vector<int> getNeighboursId(const std::vector<double>& config);

    void growTree();
    bool extend(const std::vector<double>& config);

    void rewire(const std::vector<double>& config);

    void addVertex(const std::vector<double>& config);
    void addEdge(const int id1, const int id2);
    void updateCost(int id, double cost);
    
    // bool isValidEdge(const std::vector<double>& config1, const std::vector<double>& config2);

    std::vector<std::vector<double>> findPath();

    int num_samples;
    // Epsilon: the longest distance to extend while growing the tree
    double eps;
    // If within reached_threshold, considered as goal reached
    double reached_threshold;
    // Radius to search neighbours within
    double neighbour_radius;

    bool goal_reached;
    int goal_parent;

    std::unordered_map<int, int> parent;
    std::unordered_map<int, double> costs;

    std::unordered_map<int, std::vector<double>> tree;

};


#endif