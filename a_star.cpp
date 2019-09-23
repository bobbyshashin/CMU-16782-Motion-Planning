#include "a_star.h"
#include <math.h>

void AStarPlanner::init(int start_x, int start_y, int goal_x, int goal_y, double weight) {
    open_list.clear();
    closed_list.clear();
    f_to_coordinate.clear();

    this->start_x = start_x;
    this->start_y = start_y;
    this->goal_x = goal_x;
    this->goal_y = goal_y;
    Cell start(start_x, start_y, map[start_x][start_y].cost, getHeuristic(start_x, start_y, weight));
    addCellToOpenList(std::make_pair(start_x, start_y), start);
}

void AStarPlanner::initMapSize(int x_size, int y_size) {
    map.resize(x_size);
    for (int i=0; i<x_size; ++i) {
        map[i].resize(y_size);
    }
    is_initialized = true;
}

int AStarPlanner::getHeuristic(int x, int y, double weight) {
    return (int) weight * (abs(x - goal_x) + abs(y - goal_y));
}


void AStarPlanner::initMapCell(int x, int y, int cost) {
    if (isValidCoordinate(std::make_pair(x, y))) {
        map[x][y] = MapCell(cost, true);
    }
}

void AStarPlanner::addCellToOpenList(const std::pair<int, int> coordinate, const Cell& cell) {
    f_to_coordinate.insert({cell.F, coordinate});
    open_list.insert({coordinate, cell});
}

void AStarPlanner::addCellToClosedList(const std::pair<int, int> coordinate, const Cell& cell) {
    closed_list.insert({coordinate, cell});
};

void AStarPlanner::updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int G) {
    auto& original_cell = open_list.at(coordinate);
    original_cell.parent_x = parent_x;
    original_cell.parent_y = parent_y;
    original_cell.G = G;
    original_cell.F = original_cell.H + G;
}

std::pair<std::pair<int, int>, AStarPlanner::Cell> AStarPlanner::getCoordinateCellPairWithSmallestF() {
    auto itr = f_to_coordinate.begin();
    if (itr != f_to_coordinate.end()) {
        const auto coordinate = itr->second;
        auto cell_itr = open_list.find(coordinate);
        const auto cell = cell_itr->second;
        f_to_coordinate.erase(itr);
        open_list.erase(cell_itr);
        return std::make_pair(coordinate, cell);
    }
}

// Note: may erase different element two times!

// std::pair<int, int> AStarPlanner::getCoordinateOfCellWithSmallestF() {
//     return getCoordinateCellPairWithSmallestF().first;
// }

// AStarPlanner::Cell AStarPlanner::getCellWithSmallestF() {
//     return getCoordinateCellPairWithSmallestF().second;
// }

AStarPlanner::Cell AStarPlanner::getCellFromOpenList(std::pair<int, int> coordinate) {
    const auto cell = open_list.at(coordinate);
    return cell;
}

AStarPlanner::Cell AStarPlanner::constructCell(std::pair<int, int> coordinate, int parent_x, int parent_y) {
    int x = coordinate.first;
    int y = coordinate.second;
    AStarPlanner::Cell new_cell(map[x][y].cost + map[parent_x][parent_y].cost, parent_x, parent_y);
    return new_cell;
}

bool AStarPlanner::isInOpenList(std::pair<int, int> coordinate) {
    return open_list.find(coordinate) != open_list.end();
}

bool AStarPlanner::isInClosedList(std::pair<int, int> coordinate) {
    return closed_list.find(coordinate) != closed_list.end();
}

bool AStarPlanner::hasBeenExplored(std::pair<int, int> coordinate) {
    return isInOpenList(coordinate) || isInClosedList(coordinate);

}

std::pair<int, int> AStarPlanner::getNextAction(const std::pair<int, int>& goal) {
    int x = goal.first;
    int y = goal.second;
    int counter = 0;
    while (true) {
        auto cell_itr = closed_list.find(std::make_pair(x, y));
        if (cell_itr == closed_list.end()) {
            mexPrintf("Not found! \n");
        }


        if (cell_itr->second.parent_x != start_x || cell_itr->second.parent_y != start_y) {
            x = cell_itr->second.parent_x;
            y = cell_itr->second.parent_y;
        }
        else {
            mexPrintf("Action found! \n");
            return std::make_pair(x, y);
        }
        counter++;
        mexPrintf("%d \n", counter);
    }

    // mexPrintf("Counter found! \n");

}