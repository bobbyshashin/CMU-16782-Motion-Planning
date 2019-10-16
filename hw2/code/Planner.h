#include <vector>

#define GETMAPINDEX(X, Y, XSIZE, YSIZE) (Y*XSIZE + X)

class Planner {
 public:
    Planner(int DOF, 
            double* joint_limits, 
            int map_size_x, 
            int map_size_y, 
            double* map, 
            int link_length_cells, 
            double pi) : DOF(DOF), map_size_x(map_size_x), map_size_y(map_size_y), link_length_cells(link_length_cells), pi(pi) {

                // initialize the map
                for (int i=0; i<map_size_x; ++i) {
                    std::vector<double> row;
                    for (int j=0; j<map_size_y; ++j) {
                        row.push_back(map[GETMAPINDEX(i, j, map_size_x, map_size_y)]);
                    }
                    this->map.push_back(row);
                }

                // // initialize the joint limits
                // for (int k=0; k<DOF; ++k) {
                //     this->joint_limits.push_back(joint_limits[k]);
                // }
                
            }
    // Degree of freedom
    int DOF;
    std::vector<double> joint_limits;

    int map_size_x, map_size_y;
    std::vector< std::vector<double> > map;

    bool IsValidArmConfiguration(const std::vector<double>& angles) const;
    bool IsValidLineSegment(double x0, double y0, double x1, double y1) const;

    int link_length_cells;
    double pi;
};