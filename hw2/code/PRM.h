#ifndef PRM_PLANNER_H
#define PRM_PLANNER_H

#include <unordered_map>
#include <unordered_set>
#include <map>
#include <queue>

#include "Planner.h"


class PRM_Planner : public Planner {
 public:

    PRM_Planner(int K,
        int max_successors,
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
        double goal_bias_weight) : Planner(DOF, joint_limits, start, goal, map_size_x, map_size_y, map, link_length_cells, pi, goal_bias_weight), num_samples(K), max_successors(max_successors), reached_threshold(reached_threshold), neighbour_radius(neighbour_radius) {}
    void init();

    int getNearestNeighbourId(const std::vector<double>& config);
    std::vector<int> getNeighboursId(const std::vector<double>& config);

    void buildRoadmap();

    void addVertex(const std::vector<double>& config);
    void addEdge(const int id1, const int id2);
    void updateCost(int id, double cost);
    
    std::vector<std::vector<double>> findPath();

    int num_samples;

    // If within reached_threshold, considered as goal reached
    double reached_threshold;
    // Radius to search neighbours within
    double neighbour_radius;

    bool goal_reached;
    int goal_parent;

    int max_successors;

    // int current_num_samples;

    std::unordered_map<int, std::unordered_set<int>> edges;
    std::vector<std::vector<double>> vertices;

};


#endif
