#include "RRT_Connect.h"
#include <mex.h>
#include <limits>

void RRT_Connect_Planner::init() {
    goal_reached = false;
    curr_tree_id = 1;

    addVertex(start, 1);
    addEdge(-1, 0, 1);

    addVertex(goal, 2);
    addEdge(-1, 0, 2);
}


void RRT_Connect_Planner::swapTree() {
    curr_tree_id = (curr_tree_id == 1) ? 2 : 1;
}

void RRT_Connect_Planner::growTree() {
    while ((tree1.size() + tree2.size()) < num_samples) {
        std::vector<double> q;

        q = generateRandomSample();

        bool advanced = extend(q);
        // mexPrintf("Extend!\n");

        swapTree();
        bool connected = false;
        if (advanced) {
            connected = connect(q);
            // mexPrintf("Connect!\n");
        }

        if (connected) {
            goal_reached = true;
            mexPrintf("Goal reached!\n");
            mexPrintf("Current number of samples: %d (tree1)\n", tree1.size());
            mexPrintf("Current number of samples: %d (tree2)\n", tree2.size());
            break;
        }
    }
}

bool RRT_Connect_Planner::extend(const std::vector<double>& config) {
    int nn_id = getNearestNeighbourId(config, curr_tree_id);
    if (nn_id == -1) {
        mexPrintf("Nearest neighbour ID = -1! This should not happen!\n");
    }
    
    const auto& q_near = (curr_tree_id == 1) ? tree1[nn_id] : tree2[nn_id];

    double dist = euclideanDist(q_near, config);

    std::vector<double> direction;
    for (int i=0; i<DOF; ++i) {
        direction.push_back(config[i] - q_near[i]);
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

    if (curr_tree_id == 1) {
        addEdge(nn_id, tree1.size(), 1);
    } else if (curr_tree_id == 2) {
        addEdge(nn_id, tree2.size(), 2);
    }

    addVertex(q_new, curr_tree_id);
    return true;

}

bool RRT_Connect_Planner::connect(const std::vector<double>& config) {
    int nn_id = getNearestNeighbourId(config, curr_tree_id);
    if (nn_id == -1) {
        mexPrintf("Nearest neighbour ID = -1! This should not happen!\n");
    }
    
    const auto& q_near = (curr_tree_id == 1) ? tree1[nn_id] : tree2[nn_id];

    double dist = euclideanDist(q_near, config);

    std::vector<double> direction;
    for (int i=0; i<DOF; ++i) {
        direction.push_back(config[i] - q_near[i]);
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

    if (curr_tree_id == 1) {
        addEdge(nn_id, tree1.size(), 1);
    } else if (curr_tree_id == 2) {
        addEdge(nn_id, tree2.size(), 2);
    }

    addVertex(q_new, curr_tree_id);

    if (n == interpolation_num) {
        
        connection_id1 = tree1.size()-1;
        connection_id2 = tree2.size()-1;
        return true;
    }
    return false;

}


int RRT_Connect_Planner::getNearestNeighbourId(const std::vector<double>& config, int tree_id) {
    double min_dist = std::numeric_limits<double>::max();
    int nn_id = -1;
    if (tree_id == 1) {
        for (const auto& q : tree1) {
            double dist = euclideanDist(q.second, config);
            if (dist < min_dist) {
                min_dist = dist;
                nn_id = q.first;
            }
        }
    } else if (tree_id == 2) {
        for (const auto& q : tree2) {
            double dist = euclideanDist(q.second, config);
            if (dist < min_dist) {
                min_dist = dist;
                nn_id = q.first;
            }
        } 
    }

    return nn_id;
}


void RRT_Connect_Planner::addVertex(const std::vector<double>& config, const int tree_id) {
    if (tree_id == 1) {
        int id = tree1.size();
        tree1[id] = config;

    } else if (tree_id == 2) {
        int id = tree2.size();
        tree2[id] = config;
    }
}


void RRT_Connect_Planner::addEdge(const int id1, const int id2, const int tree_id) {
    if (tree_id == 1) {
        parent1[id2] = id1;
    }
    else if (tree_id == 2) {
        parent2[id2] = id1;
    }

}

std::vector<std::vector<double>> RRT_Connect_Planner::interpolatePath(std::vector<std::vector<double>>& path) {
    std::vector<std::vector<double>> interpolated_path;
    interpolated_path.push_back(path[0]);

    for (int k=0; k<path.size()-1; ++k) {
        std::vector<double> direction;
        for (int i=0; i<DOF; ++i) {
            direction.push_back(path[k+1][i] - path[k][i]);
        }

        std::vector<double> inter(path[k]);
        int n = 1;
        int interpolation_num = 5;
        while (n <= interpolation_num) {
            for (int i=0; i<DOF; ++i) {
                inter[i] = path[k][i] + ((double)n / (double)interpolation_num) * direction[i];
            }
            ++n;
            interpolated_path.push_back(inter);
        }

    }

    return interpolated_path;
    

}

std::vector<std::vector<double>> RRT_Connect_Planner::findPath() {
    int parent_id = connection_id2;
    std::vector<int> path2_with_ids;
    while (parent_id != -1) {
        path2_with_ids.push_back(parent_id);
        parent_id = parent2[parent_id];
    }

    parent_id = connection_id1;
    std::vector<int> reversed_path1_with_ids;
    while (parent_id != -1) {
        reversed_path1_with_ids.push_back(parent_id);
        parent_id = parent1[parent_id];
    }

    int path_length = reversed_path1_with_ids.size() + path2_with_ids.size();
    mexPrintf("Found path length: %d\n", path_length);

    std::vector<int> path1_with_ids;
    for (int i=reversed_path1_with_ids.size()-1; i>=0; --i) {
        path1_with_ids.push_back(reversed_path1_with_ids[i]);
    }

    std::vector<std::vector<double>> path;
    for (const auto& id : path1_with_ids) {
        path.push_back(tree1[id]);

    }
    for (const auto& id : path2_with_ids) {
        path.push_back(tree2[id]);
    }
    
    path.push_back(goal);
    
    return path;

}

