#include "dijkstra.h"
#include <math.h>

int GlobalDijkstraPlanner::manhattanDistance(std::pair<int, int> a, std::pair<int, int> b) {
    return abs(a.first - b.first) + abs(a.second - b.second);
}

void GlobalDijkstraPlanner::initMapSize(int x_size, int y_size) {
    map.resize(x_size);
    for (int i=0; i<x_size; ++i) {
        map[i].resize(y_size);
    }
    total_num_cells = x_size * y_size;
    is_initialized = true;
}

void GlobalDijkstraPlanner::initMapCell(std::pair<int, int> coordinate, int cost) {
    if (isValidCoordinate(coordinate)) {
        int x = coordinate.first;
        int y = coordinate.second;
        map[x][y] = MapCell(manhattanDistance(coordinate, robot_start), manhattanDistance(coordinate, target_start), cost, true);
    }
}

void GlobalDijkstraPlanner::initTargetTrajectory(std::vector<std::pair<int, int>> traj) {
    for (int i=0; i<traj.size(); ++i) {
        // ordered_target_trajectory.push_back(traj[i]);
        target_trajectory.insert({traj[i], i});
    }
}

void GlobalDijkstraPlanner::init(std::pair<int, int> start) {
    open_list.clear();
    closed_list.clear();
    cost_to_coordinate.clear();

    // for (const auto& p : target_trajectory) {
    //     // const auto& p = target_trajectory[i];
    //     int x = p.first.first;
    //     int y = p.first.second;
    //     Cell cell(map[x][y].cost);
    //     addCellToOpenList(std::make_pair(x, y), cell);
    // }
    this->start = start;
    int x = start.first;
    int y = start.second;
    Cell cell(map[x][y].cost);
    addCellToOpenList(start, cell);
}

void GlobalDijkstraPlanner::addCellToOpenList(const std::pair<int, int> coordinate, const Cell& cell) {
    cost_to_coordinate.insert({cell.cost, coordinate});
    open_list.insert({coordinate, cell});
}

void GlobalDijkstraPlanner::addCellToClosedList(const std::pair<int, int> coordinate, const Cell& cell) {
    closed_list.insert({coordinate, cell});
};

void GlobalDijkstraPlanner::updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int cost) {
    auto& original_cell = open_list.at(coordinate);
    original_cell.parent_x = parent_x;
    original_cell.parent_y = parent_y;
    original_cell.cost = cost;
}

std::pair<std::pair<int, int>, GlobalDijkstraPlanner::Cell> GlobalDijkstraPlanner::getCoordinateCellPairWithSmallestF() {
    auto itr = cost_to_coordinate.begin();
    if (itr != cost_to_coordinate.end()) {
        const auto coordinate = itr->second;
        auto cell_itr = open_list.find(coordinate);
        const auto cell = cell_itr->second;
        cost_to_coordinate.erase(itr);
        open_list.erase(cell_itr);
        return std::make_pair(coordinate, cell);
    }
}

GlobalDijkstraPlanner::Cell GlobalDijkstraPlanner::getCellFromOpenList(std::pair<int, int> coordinate) {
    const auto cell = open_list.at(coordinate);
    return cell;
}

GlobalDijkstraPlanner::Cell GlobalDijkstraPlanner::constructCell(std::pair<int, int> coordinate, int parent_x, int parent_y) {
    int x = coordinate.first;
    int y = coordinate.second;
    GlobalDijkstraPlanner::Cell new_cell(map[x][y].cost + closed_list.at(std::make_pair(parent_x, parent_y)).cost, parent_x, parent_y);
    return new_cell;
}

bool GlobalDijkstraPlanner::isInOpenList(std::pair<int, int> coordinate) {
    return open_list.find(coordinate) != open_list.end();
}

bool GlobalDijkstraPlanner::isInClosedList(std::pair<int, int> coordinate) {
    return closed_list.find(coordinate) != closed_list.end();
}

bool GlobalDijkstraPlanner::hasBeenExplored(std::pair<int, int> coordinate) {
    return isInOpenList(coordinate) || isInClosedList(coordinate);

}

GlobalDijkstraPlanner::Path GlobalDijkstraPlanner::findPath(std::pair<int, int> coordinate) {

    // Path p;
    // auto curr = coordinate;
    // int position_on_target_traj = isOnTargetTrajectory(coordinate);
    // while (position_on_target_traj == -1) {
    //     if (!isInClosedList(curr)) {
    //         mexPrintf("Unable to find path from Dijkstra! \n");
    //     }

    //     auto& cell = closed_list.at(curr);
    //     curr.first = cell.parent_x;
    //     curr.second = cell.parent_y;
    //     p.path.push_back(curr);
    //     position_on_target_traj = isOnTargetTrajectory(curr);
    // }

    // p.path.push_back(curr);
    // p.position_on_target_traj = position_on_target_traj;
    // p.length = p.path.size();
    // return p;
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


        auto& cell = closed_list.at(curr);
        p.path.push_back(curr);
        curr.first = cell.parent_x;
        curr.second = cell.parent_y;

    }

    // p.path.push_back(curr);
    p.cost = closed_list.at(coordinate).cost;
    p.length = p.path.size();
    return p;
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
