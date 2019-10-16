#include <unordered_map>
#include <map>
#include <queue>


class PRM {
 public:

    void init(int num_samples);

    std::vector<double> generateRandomSample(); 
    void buildRoadmap();

    int num_samples;


};
