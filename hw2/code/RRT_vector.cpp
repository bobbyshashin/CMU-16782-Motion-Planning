#include "RRT_vector.h"
#include <mex.h>
#include <limits>

void RRT_Planner::init() {
    goal_reached = false;
    addVertex(start);
    addEdge(-1, 0);
}


void RRT_Planner::growTree() {
    while (tree.size() < num_samples) {
        std::vector<double> q;
        if (chooseGoal()) {
            // mexPrintf("Choose goal \n");
            q = goal;
        } else {
            // mexPrintf("Choose randomly \n");
            q = generateRandomSample();
        }

        extendWithInterpolation(q);
        // extend(q);

        if (goal_reached) {
            mexPrintf("Goal reached! Current number of samples: %d\n", tree.size());
            break;
        }
    }
}

void RRT_Planner::extendWithInterpolation(const std::vector<double>& config) {
    int nn_id = getNearestNeighbourId(config);
    if (nn_id == -1) {
        mexPrintf("Nearest neighbour ID = -1! This should not happen!\n");
    }
    
    const auto& q_near = tree[nn_id];

    double dist = euclideanDist(q_near, config);

    // if the tree is close enough and the new sample is valid
    // then connect the sample to the tree
    if ((dist <= eps) && IsValidArmConfiguration(config)) {
        addEdge(nn_id, tree.size());
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
                    return;
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
        addEdge(nn_id, tree.size());
        addVertex(q_new);
    }
}

void RRT_Planner::extend(const std::vector<double>& config) {
    int nn_id = getNearestNeighbourId(config);
    if (nn_id == -1) {
        mexPrintf("Nearest neighbour ID = -1! This should not happen!\n");
    }
    
    const auto& q_near = tree[nn_id];

    double dist = euclideanDist(q_near, config);

    // if the tree is close enough and the new sample is valid
    // then connect the sample to the tree
    if ((dist <= eps) && IsValidArmConfiguration(config)) {
        addEdge(nn_id, tree.size());
        addVertex(config);
    } else {
        std::vector<double> direction;
        double ratio = eps / dist;
        for (int i=0; i<DOF; ++i) {
            direction.push_back((config[i] - q_near[i]) * ratio);
        }

        std::vector<double> q_new(q_near);
        for (int i=0; i<DOF; ++i) {
            q_new[i] = q_near[i] + direction[i];
        }
        // mexPrintf("%f\n", euclideanDist(q_new, q_near));
        if (IsValidArmConfiguration(q_new)) {
            addEdge(nn_id, tree.size());
            addVertex(q_new);
        }
    }
}

int RRT_Planner::getNearestNeighbourId(const std::vector<double>& config) {
    double min_dist = std::numeric_limits<double>::max();
    int nn_id = -1;
    for (int i=0; i<tree.size(); ++i) {
        double dist = euclideanDist(tree[i], config);
        if (dist < min_dist) {
            min_dist = dist;
            nn_id = i;
        }
    }
    return nn_id;
}


void RRT_Planner::addVertex(const std::vector<double>& config) {
    int id = tree.size();
    // tree[id] = config;
    tree.push_back(config);
    goal_reached = (euclideanDist(config, goal) < reached_threshold);
    if (goal_reached) {
        goal_parent = id;
    }
}


void RRT_Planner::addEdge(const int id1, const int id2) {
    // parent[id2] = id1; 
    parent.push_back(id1);
}

std::vector<std::vector<double>> RRT_Planner::findPath() {
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