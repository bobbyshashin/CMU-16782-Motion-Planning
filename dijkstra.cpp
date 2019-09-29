#include "dijkstra.h"
#include <mex.h>

void Dijkstra::initMapSize(int x_size, int y_size) {
    map.resize(x_size);
    cost_map.resize(x_size);
    parents_x.resize(x_size);
    parents_y.resize(x_size);
    visited.resize(x_size);
    explored.resize(x_size);

    for (int i=0; i<x_size; ++i) {
        map[i].resize(y_size);
        cost_map[i].resize(y_size);
        parents_x[i].resize(y_size);
        parents_y[i].resize(y_size);
        visited[i].resize(y_size);
        explored[i].resize(y_size);
        for (int j=0; j<y_size; ++j) {
            cost_map[i][j] = INT_MAX;
            parents_x[i][j] = -1;
            parents_y[i][j] = -1;
            visited[i][j] = false;
            explored[i][j] = false;
        }
    }

    map_size_x = x_size;
    map_size_y = y_size;
    is_initialized = true;
}

void Dijkstra::init(int start_x, int start_y, int collision_threshold) {

    this->start_x = start_x;
    this->start_y = start_y;
    this->collision_threshold = collision_threshold;

    Cell start(start_x, start_y, map[start_x][start_y]);
    addToOpenList(start);
}

void Dijkstra::initTargetTrajectory(std::vector<std::pair<int, int>> traj) {
    for (int i=0; i<traj.size(); ++i) {
        target_trajectory.insert({traj[i], i});
    }
}


bool Dijkstra::isValid(int x, int y) {

    return (x >= 0 && 
            y >= 0 &&
            x < map_size_x && 
            y < map_size_y && 
            map[x][y] < collision_threshold);
}

void Dijkstra::addToOpenList(const Cell cell) {
    open_list.push(cell);

    cost_map[cell.x][cell.y] = cell.cost;
    visited[cell.x][cell.y] = true;
    parents_x[cell.x][cell.y] = cell.parent_x;
    parents_y[cell.x][cell.y] = cell.parent_y;
}

void Dijkstra::addToClosedList(const Cell cell) {
    closed_list.insert({std::make_pair(cell.x, cell.y), cell});
    explored[cell.x][cell.y] = true;
    // set visited[cell.x][cell.y] = false; here?

}

Dijkstra::Cell Dijkstra::extractFromOpenList() {

    if (!open_list.empty()) {
        const auto& c = open_list.top();
        // mexPrintf("Cost: %d \n", c.cost);
        Cell cell(c.x, c.y, c.cost, c.parent_x, c.parent_y, c.step_distance);
        open_list.pop();
        return cell;
    }

}

Dijkstra::Path Dijkstra::findPath(int x0, int y0) {
    Path path;
    // std::pair<int, int> xy = std::make_pair(x, y);
    int x = x0;
    int y = y0;

    path.cost = closed_list.at(std::make_pair(x, y)).cost;
    path.path.push_back(std::make_pair(x, y));

    while (x != start_x || y != start_y) {
        if (!isInClosedList(x, y)) {
            mexPrintf("Unable to find path from Dijkstra! \n");
            path.cost = -1;
            return path;
        }
        // auto& cell = closed_list.at(std::make_pair(x, y));
        // x = cell.parent_x;
        // y = cell.parent_y;
        int px = parents_x[x][y];
        int py = parents_y[x][y];
        x = px;
        y = py;

        path.path.push_back(std::make_pair(x, y));
    }

    path.length = path.path.size();
    // mexPrintf("path length %d\n", path.length);
    return path;
}

void Dijkstra::generatePath() {
    // int counter = 0;
    for (const auto& point : target_trajectory) {
        // if (counter % 100 == 0) {
        //     mexPrintf("Processed %d points on trajectory\n", counter);
        // }
        // counter++;

        int x = point.first.first;
        int y = point.first.second;
        int target_arrival_time = point.second;

        if (!isInClosedList(x, y)) {
            continue;
        }

        const auto& path = findPath(x, y);
        if (path.cost == -1) {
            // invalid path, skip
            continue;
        }
        int wait_time = target_arrival_time - path.length;
        if (wait_time > 1) {
            evaluated_paths.insert({path.cost + wait_time * map[x][y], path});
        }

    }

}

void Dijkstra::search() {

    while (!isCompleted()) {
        // if (closed_list.size() % 1000 == 0) {
        //     mexPrintf("Dijkstra explored: %d\n", closed_list.size());
        // }

        const auto& curr_cell = extractFromOpenList();
        addToClosedList(curr_cell);
        int curr_x = curr_cell.x;
        int curr_y = curr_cell.y;

        // expand the surrounding cells (8-connected grid)
        for (int i=0; i<8; ++i) {
            int new_x = curr_x + dX[i];
            int new_y = curr_y + dY[i];

            if (isValid(new_x, new_y) && !isInClosedList(new_x, new_y)) {
                // new_cost = parent_accumulated_cost + edge_cost
                int new_cost = cost_map[curr_x][curr_y] + map[new_x][new_y];
                const bool lower_cost = new_cost < cost_map[new_x][new_y];

                if (!isInOpenList(new_x, new_y) || lower_cost) {
                    Cell new_cell(new_x, new_y, new_cost, curr_x, curr_y, curr_cell.step_distance + 1);
                    addToOpenList(new_cell);

                }
            }
        }
    }
    mexPrintf("Search completed!\n");

    generatePath();
    mexPrintf("Path generated!\n");
}
