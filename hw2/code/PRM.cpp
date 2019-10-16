#include "PRM.h"
#include <mex.h>

void PRM::buildRoadmap() {
    // add start and goal into the roadmap

    int i = 0;
    while (i < num_samples) {
    
        // generate a random sample

        // perform collision checking for this sample

        // if free, then:
            // add the vertex

            //   ++i;

            // find the kNN of this sample, loop through them

            // if not belong to the same component, then connect them, add an edge to the graph

    }


}

std::vector<double> PRM::generateRandomSample() {




}