#include "forward_dijkstra.h"
#include <math.h>

int GlobalDijkstraPlanner::manhattanDistance(std::pair<int, int> a, std::pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

void GlobalDijkstraPlanner::initMapSize(int x_size, int y_size) {
    map.resize(x_size);
    cost_map.resize(x_size);
    parents.resize(x_size);
    open_list_flag.resize(x_size);
    closed_list_flag.resize(x_size);

    for (int i=0; i<x_size; ++i) {
        map[i].resize(y_size);
        cost_map[i].resize(y_size);
        parents[i].resize(y_size);
        open_list_flag[i].resize(y_size);
        closed_list_flag[i].resize(y_size);

        for (int j=0; j<y_size; ++j) {
            parents[i][j] = (std::make_pair(-1, -1));
            open_list_flag[i][j] = false;
            closed_list_flag[i][j] = false;
            cost_map[i][j] = INT_MAX;
        }
    }
    total_num_cells = x_size * y_size;
    // cost_map.assign(x_size, std::vector<int>(y_size, 0));
    // parents.assign(x_size, std::vector<std::pair<int, int>>(y_size, std::make_pair(-1, -1)));
    // open_list_flag.assign(x_size, std::vector<bool>(y_size, false));
    // closed_list_flag.assign(x_size, std::vector<bool>(y_size, false));
    is_initialized = true;
}

void GlobalDijkstraPlanner::initMapCell(std::pair<int, int> coordinate, int cost) {
    if (isValidCoordinate(coordinate)) {
        int x = coordinate.first;
        int y = coordinate.second;
        // map[x][y] = MapCell(cost);
    }
}

void GlobalDijkstraPlanner::initTargetTrajectory(std::vector<std::pair<int, int>> traj) {
    for (int i=0; i<traj.size(); ++i) {
        // ordered_target_trajectory.push_back(traj[i]);
        target_trajectory.insert({traj[i], i});
    }
}

void GlobalDijkstraPlanner::init(std::pair<int, int> start) {
    // open_list.clear();
    // closed_list.clear();
    // cost_to_coordinate.clear();

    this->start = start;

    Cell start_cell(map[start.first][start.second]);
    // least_cost = start_cell.cost;
    cost_map[start.first][start.second] = start_cell.cost;
    // cost_to_coordinate.push(CostCoordinate(cell.cost, coordinate));
    addCellToOpenList(start, start_cell);
}

void GlobalDijkstraPlanner::addCellToOpenList(const std::pair<int, int> coordinate, const Cell& cell) {
    // cost_to_coordinate.insert({cell.cost, coordinate});
    cost_to_coordinate.push(CostCoordinate(cell.cost, coordinate));
    // open_list.insert({coordinate, cell});
    cost_map[coordinate.first][coordinate.second] = cell.cost;
    open_list_flag[coordinate.first][coordinate.second] = true;
}

void GlobalDijkstraPlanner::addCellToClosedList(const std::pair<int, int> coordinate, const Cell& cell) {
    closed_list.insert({coordinate, cell});
    closed_list_flag[coordinate.first][coordinate.second] = true;
};

void GlobalDijkstraPlanner::updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int cost, int step_distance) {
    // auto& original_cell = open_list.at(coordinate);
    // original_cell.parent_x = parent_x;
    // original_cell.parent_y = parent_y;
    // original_cell.cost = cost;
    // original_cell.step_distance = step_distance;
}

std::pair<std::pair<int, int>, GlobalDijkstraPlanner::Cell> GlobalDijkstraPlanner::getCoordinateCellPairWithSmallestF() {
    // auto itr = cost_to_coordinate.begin();
    // if (itr != cost_to_coordinate.end()) {
    //     const auto coordinate = itr->second;
    //     auto cell_itr = open_list.find(coordinate);
    //     const auto cell = cell_itr->second;
    //     cost_to_coordinate.erase(itr);
    //     open_list.erase(cell_itr);
    //     open_list_flag[coordinate.first][coordinate.second] = false;
    //     return std::make_pair(coordinate, cell);
    // }
    auto cost_coordinate = cost_to_coordinate.top();

    // auto cell_itr = open_list.find(cost_coordinate.coordinate);
    // const auto cell = cell_itr->second;
    auto cell = GlobalDijkstraPlanner::Cell(cost_coordinate.cost, parents[cost_coordinate.coordinate.first][cost_coordinate.coordinate.second].first, parents[cost_coordinate.coordinate.first][cost_coordinate.coordinate.second].second);

    // open_list.erase(cell_itr);
    open_list_flag[cost_coordinate.coordinate.first][cost_coordinate.coordinate.second] = false;

    cost_to_coordinate.pop();
    return std::make_pair(cost_coordinate.coordinate, cell);
}

GlobalDijkstraPlanner::Cell GlobalDijkstraPlanner::getCellFromOpenList(std::pair<int, int> coordinate) {
    const auto cell = open_list.at(coordinate);
    return cell;
}

GlobalDijkstraPlanner::Cell GlobalDijkstraPlanner::constructCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int step_distance) {
    int x = coordinate.first;
    int y = coordinate.second;
    GlobalDijkstraPlanner::Cell new_cell(map[x][y] + cost_map[parent_x][parent_y], parent_x, parent_y, step_distance);
    parents[x][y] = std::make_pair(parent_x, parent_y);
    return new_cell;
}

bool GlobalDijkstraPlanner::isInOpenList(std::pair<int, int> coordinate) {
    // return open_list.find(coordinate) != open_list.end();
    return open_list_flag[coordinate.first][coordinate.second];
}

bool GlobalDijkstraPlanner::isInClosedList(std::pair<int, int> coordinate) {
    // return closed_list.find(coordinate) != closed_list.end();
    return closed_list_flag[coordinate.first][coordinate.second];
}

bool GlobalDijkstraPlanner::hasBeenExplored(std::pair<int, int> coordinate) {
    return isInOpenList(coordinate) || isInClosedList(coordinate);

}

GlobalDijkstraPlanner::Path GlobalDijkstraPlanner::findPath(std::pair<int, int> coordinate) {

    Path p;
    std::pair<int, int> curr;
    curr.first = coordinate.first;
    curr.second = coordinate.second;
    while (curr.first != start.first || curr.second != start.second) {
        if (!isInClosedList(curr)) {
            mexPrintf("Curr: %d, %d \n", curr.first, curr.second);
            mexPrintf("Closed list size: %d \n", closed_list.size());
            mexPrintf("Unable to find path from Dijkstra! \n");
            break;
        }
        curr = parents[curr.first][curr.second];
        p.path.push_back(curr);
        // curr.first = cell.parent_x;
        // curr.second = cell.parent_y;

    }

    // p.path.push_back(curr);
    p.cost = closed_list.at(coordinate).cost;
    p.length = p.path.size();
    return p;
    // while (curr.first != start.first || curr.second != start.second) {
    //     if (!isInClosedList(curr)) {
    //         mexPrintf("Curr: %d, %d \n", curr.first, curr.second);
    //         mexPrintf("Closed list size: %d \n", closed_list.size());
    //         mexPrintf("Unable to find path from Dijkstra! \n");
    //         break;
    //     }


    //     auto& cell = closed_list.at(curr);
    //     p.path.push_back(curr);
    //     curr.first = cell.parent_x;
    //     curr.second = cell.parent_y;

    // }

    // // p.path.push_back(curr);
    // p.cost = closed_list.at(coordinate).cost;
    // p.length = p.path.size();
    // return p;
}

GlobalDijkstraPlanner::Path GlobalDijkstraPlanner::findBestPath(std::pair<int, int> coordinate, int curr_time) {

    Path best_path;
    int min_cost = INT_MAX;

    for (const auto& point : target_trajectory) {
        auto goal = point.first;
        auto dist = point.second;
        if (dist < curr_time) {
            continue;
        }

        if (dist - curr_time > target_trajectory.size() / 10) {
            continue;
        }

        Path p;
        auto curr = coordinate;
        int total_cost = 0;
        // while (curr.first != goal.first || curr.second != goal.second) {
        //     // mexPrintf("Curr: %d, %d \n", curr.first, curr.second);
        //     // comment this out to speed up (maybe a tiny little bit)
        //     if (!isInClosedList(curr)) {
        //         mexPrintf("Unable to find path from Dijkstra! \n");
        //         break;
        //     }

        //     auto& cell = closed_list.at(curr);
        //     curr.first = cell.parent_x;
        //     curr.second = cell.parent_y;
        //     total_cost += cell.cost;
        //     p.path.push_back(curr);
        // }

        // p.path.push_back(curr);
        // p.length = p.path.size();

        int wait_time = dist - curr_time - p.length;
        if (wait_time < 0) {
            // means target will pass here
            continue;
        }

        total_cost += wait_time * closed_list.at(goal).cost;
        if (total_cost < min_cost) {
            min_cost = total_cost;
            best_path = p;
        }
        
    }

    return best_path;
   
}
