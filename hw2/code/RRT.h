#include <unordered_map>
#include <map>
#include <queue>

#include "Planner.h"


class RRT : public Planner {
 public:

    RRT(double eps,
        int DOF, 
        double* joint_limits,
        double* start,
        double* goal,
        int map_size_x, 
        int map_size_y, 
        double* map, 
        int link_length_cells, 
        double pi) : Planner(DOF, joint_limits, start, goal, map_size_x, map_size_y, map, link_length_cells, pi), eps(eps) {}
    void init();

    std::vector<double> generateRandomSample(); 
    int getNearestNeighbourId();


    void extend()
    void growTree();


    // Epsilon: the longest distance to extend while growing the tree
    double eps;


};