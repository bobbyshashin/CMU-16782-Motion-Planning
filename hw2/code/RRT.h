#include <unordered_map>
#include <map>
#include <queue>

#include "Planner.h"


class RRT : public Planner {
 public:

    
    void init(int num_samples);

    std::vector<double> generateRandomSample(); 
    void growTree();


    // Epsilon: the longest distance to extend while growing the tree
    double eps;


};