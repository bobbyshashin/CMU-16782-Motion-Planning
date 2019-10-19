#include "RRT_Star.h"
#include <mex.h>
#include <limits>

void RRT_Star_Planner::init() {
    goal_reached = false;
    addVertex(start);
    addEdge(-1, 0);
    updateCost(0, 0);
}


void RRT_Star_Planner::growTree() {
    while (tree.size() < num_samples) {
        std::vector<double> q;
        if (chooseGoal()) {
            q = goal;
        } else {
            q = generateRandomSample();
        }

        bool extended = extend(q);
        if (extended) {
            // get q_new, which is the one just added
            auto q_new = tree[tree.size()-1];
            rewire(q_new);
        }

        if (goal_reached) {
            mexPrintf("Goal reached! Current number of samples: %d\n", tree.size());
            break;
        }
    }
}

bool RRT_Star_Planner::isValidEdge(const std::vector<double>& config1, const std::vector<double>& config2) {
    std::vector<double> direction;
    for (int i=0; i<DOF; ++i) {
        direction.push_back(config2[i] - config1[i]);
    }

    std::vector<double> inter(config1);
    int n = 1;
    int interpolation_num = 100;
    while (n <= interpolation_num) {
        for (int i=0; i<DOF; ++i) {
            inter[i] = config1[i] + ((double)n / (double)interpolation_num) * direction[i];
        }
        if (!IsValidArmConfiguration(inter)) {
           return false;
        }
        ++n;
    }
    return true;

}


bool RRT_Star_Planner::extend(const std::vector<double>& config) {
    int nn_id = getNearestNeighbourId(config);
    if (nn_id == -1) {
        mexPrintf("Nearest neighbour ID = -1! This should not happen!\n");
    }
    
    const auto& q_near = tree[nn_id];

    double dist = euclideanDist(q_near, config);

    // if the tree is close enough and the new sample is valid
    // then connect the sample to the tree
    if ((dist <= eps) && IsValidArmConfiguration(config)) {
        // addEdge(nn_id, tree.size());
        updateCost(tree.size(), costs[nn_id] + dist);
        addVertex(config);
    } else {
        std::vector<double> direction;
        double ratio = eps / dist;
        for (int i=0; i<DOF; ++i) {
            direction.push_back((config[i] - q_near[i]) * ratio);
        }

        std::vector<double> q_new(q_near);
        int n = 1;
        int interpolation_num = 100;
        while (n <= interpolation_num) {
            for (int i=0; i<DOF; ++i) {
                q_new[i] = q_near[i] + ((double)n / (double)interpolation_num) * direction[i];
            }
            if (!IsValidArmConfiguration(q_new)) {
                if (n == 1) {
                    // q_near is already on the edge of an obstacle, cannot extend anymore
                    return false;
                } else {
                    // we encountered an obstacle
                    // go back to the previous valid configuration, and add that point
                    for (int i=0; i<DOF; ++i) {
                        q_new[i] = q_near[i] + ((double) (n-1) / (double)interpolation_num) * direction[i];
                    }
                    break;
                }
            }
            ++n;
        }
        // addEdge(nn_id, tree.size());
        updateCost(tree.size(), costs[nn_id] + euclideanDist(q_new, q_near));
        addVertex(q_new);
    }
    return true;
}

void RRT_Star_Planner::rewire(const std::vector<double>& config) {
    // speed can be optimized here, no need to recompute in the second for loop
    int q_new_id = tree.size()-1;
    const auto& neighbours = getNeighboursId(config);
    double min_cost = costs[q_new_id];
    int min_id = q_new_id;
    for (const auto& n : neighbours) {
        if (isValidEdge(tree[n], config)) {
            double cost = costs[n] + euclideanDist(tree[n], config);
            if (cost <= min_cost) {
                min_cost = cost;
                min_id = n;
            }
        }
    }
    if (min_id == -1) {
        mexPrintf("Min cost neighbour ID = -1! This should not happen!\n");
    }
    addEdge(min_id, q_new_id);
    updateCost(q_new_id, min_cost);

    for (const auto& n : neighbours) {
        if (n != min_id) {
            // skip x_min
            if (isValidEdge(config, tree[n])) {
                if (costs[n] > min_cost + euclideanDist(tree[n], config)) {
                    parent[n] = q_new_id;
                }
            }

        }

    }

}

int RRT_Star_Planner::getNearestNeighbourId(const std::vector<double>& config) {
    double min_dist = std::numeric_limits<double>::max();
    int nn_id = -1;
    for (const auto& q : tree) {
        double dist = euclideanDist(q.second, config);
        if (dist < min_dist) {
            min_dist = dist;
            nn_id = q.first;
        }
    }
    return nn_id;
}

std::vector<int> RRT_Star_Planner::getNeighboursId(const std::vector<double>& config) {
    std::vector<int> neighbours;
    for (const auto& q : tree) {
        double dist = euclideanDist(q.second, config);
        if (dist <= neighbour_radius) {
            if (dist != 0) {
                // don't add itself
                neighbours.push_back(q.first);
            }
                
        }
    }
    return neighbours;
}


void RRT_Star_Planner::addVertex(const std::vector<double>& config) {
    int id = tree.size();
    tree[id] = config;
    goal_reached = (euclideanDist(config, goal) < reached_threshold);
    if (goal_reached) {
        goal_parent = id;
    }
}


void RRT_Star_Planner::addEdge(const int id1, const int id2) {
    parent[id2] = id1; 
}

void RRT_Star_Planner::updateCost(int id, double cost) {
    costs[id] = cost;
}

std::vector<std::vector<double>> RRT_Star_Planner::findPath() {
    int parent_id = goal_parent;
    std::vector<int> reversed_path_with_ids;
    while (parent_id != -1) {
        reversed_path_with_ids.push_back(parent_id);
        parent_id = parent[parent_id];
    }

    // include goal
    int path_length = reversed_path_with_ids.size() + 1;
    mexPrintf("Found path length: %d\n", path_length);

    std::vector<int> path_with_ids;
    for (int i=reversed_path_with_ids.size()-1; i>=0; --i) {
        path_with_ids.push_back(reversed_path_with_ids[i]);
    }

    std::vector<std::vector<double>> path;
    for (const auto& id : path_with_ids) {
        path.push_back(tree[id]);
    }
    path.push_back(goal);
    
    return path;
}