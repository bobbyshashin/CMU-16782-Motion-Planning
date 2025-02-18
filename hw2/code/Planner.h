#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <stdlib.h>
#include <time.h> 
#include <math.h>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

class Planner {
 public:
    Planner(int DOF, 
            double* joint_limits, 
            double* start,
            double* goal,
            int map_size_x, 
            int map_size_y, 
            double* map, 
            int link_length_cells, 
            double pi,
            double goal_bias_weight) : DOF(DOF), map_size_x(map_size_x), map_size_y(map_size_y), link_length_cells(link_length_cells), pi(pi), goal_bias_weight(goal_bias_weight) {

                // initialize the map
                for (int i=0; i<map_size_x; ++i) {
                    std::vector<double> row;
                    for (int j=0; j<map_size_y; ++j) {
                        row.push_back(map[GETMAPINDEX(i, j, map_size_x, map_size_y)]);
                    }
                    this->map.push_back(row);
                }

                // initialize the joint limits, start and goal configurations
                for (int k=0; k<DOF; ++k) {
                    // this->joint_limits.push_back(joint_limits[k]);
                    this->start.push_back(start[k]);
                    this->goal.push_back(goal[k]);
                }

                // initialize the random number generator seed
                srand(time(0));
                
            }
    
    std::vector<double> joint_limits;
    std::vector<double> start;
    std::vector<double> goal;

    // Map parameters
    int map_size_x, map_size_y;
    std::vector< std::vector<double> > map;

    // Collision checkers
    bool IsValidArmConfiguration(const std::vector<double>& angles) const;
    bool IsValidLineSegment(double x0, double y0, double x1, double y1) const;

    bool isValidEdge(const std::vector<double>& config1, const std::vector<double>& config2, int interpolation_num = 100);
    
    std::vector<double> generateRandomSample();
    bool chooseGoal();
    double euclideanDist(std::vector<double> config1, std::vector<double> config2);
    // Degree of freedom
    int DOF;

    int link_length_cells;
    double pi;

    double goal_bias_weight;
};

#endif