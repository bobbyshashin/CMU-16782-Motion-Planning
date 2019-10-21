#include "PRM.h"
#include <mex.h>

void PRM_Planner::init() {


}

void PRM_Planner::buildRoadmap() {
    // add start and goal into the roadmap
    bool flag = true;

    while (vertices.size() < num_samples) {
        // generate a random sample
        const auto& q = generateRandomSample();

        if (IsValidArmConfiguration(q)) {
            int new_id = vertices.size();
            addVertex(q);

            if (flag && goal_reached) {
                // flag = false;
                if (new_id % 1000 == 0)
                    mexPrintf("Goal reached! Current number of samples: %d \n", new_id);
            }
            
            const auto& neighbour_ids = getNeighboursId(q);

            for (const auto& id : neighbour_ids) {
                // bool num_successors = edges[id].size() < max_successors;
                bool num_successors = true;

                if (num_successors && isValidEdge(vertices[id], q)) {
                    addEdge(id, new_id);
                }
            }
        }

    }


}


void PRM_Planner::addVertex(const std::vector<double>& config) {
    int id = vertices.size();
    vertices.push_back(config);

    // maybe collision checking with interpolation between config and goal here?
    goal_reached = (euclideanDist(config, goal) < reached_threshold);
    if (goal_reached) {
        goal_parent = id;
    }
}

void PRM_Planner::addEdge(const int id1, const int id2) {
    auto a = edges.find(id1);
    auto b = edges.find(id2);

    if (a == edges.end()) {
        std::unordered_set<int> new_edge({id2});
        edges[id1] = new_edge;
    } else {
        a->second.insert(id2);
    }

    if (b == edges.end()) {
        std::unordered_set<int> new_edge({id1});
        edges[id2] = new_edge;
    } else {
        b->second.insert(id1);
    }


}

std::vector<int> PRM_Planner::getNeighboursId(const std::vector<double>& config) {
    std::vector<int> neighbours_id;
    for (int i=0; i<vertices.size(); ++i) {
        double dist = euclideanDist(vertices[i], config);
        if (dist <= neighbour_radius) {
            if (dist != 0) {
                // don't add itself
                neighbours_id.push_back(i);
            }
                
        }
    }
    return neighbours_id;
}
