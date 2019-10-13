#include <unordered_map>
#include <map>
#include <utility>
#include <vector>
#include <mex.h>

class GlobalDijkstraPlanner {
 public:
    struct Cell {
        int parent_x, parent_y;
        double cost;

        Cell (int cost, int parent_x = -1, int parent_y = -1) : 
            cost(cost), parent_x(parent_x), parent_y(parent_y) {}
    };

    struct MapCell {
        int cost;
        int start_distance;
        int goal_distance;

        bool is_initialized;
        MapCell() : cost(0), is_initialized(false) {}
        MapCell(int start_distance, int goal_distance, int cost, bool is_initialized=false) : start_distance(start_distance), goal_distance(goal_distance), cost(cost), is_initialized(is_initialized) {
        }
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    struct Path {
        std::vector<std::pair<int, int>> path;
        int length;
        int cost;
        int position_on_target_traj;
    };

 public:
    void initMapSize(int x_size, int y_size);
    void initMapCell(std::pair<int, int> coordinate, int cost);
    void initTargetTrajectory(std::vector<std::pair<int, int>> traj);
    void initParam(std::pair<int, int> robot_start, std::pair<int, int> target_start, int collision_thres) {
        this->robot_start = robot_start;
        this->target_start = target_start;
        collision_threshold = collision_thres;
    }

    void init(std::pair<int, int> start);

    inline bool isInitialized() const { return is_initialized; }
    inline bool isCompleted() const { return open_list.empty(); }

    inline bool isValidCoordinate(std::pair<int, int> coordinate) { return (coordinate.first >= 0 && coordinate.first < map.size() && coordinate.second >= 0 && coordinate.second < map[0].size() && map[coordinate.first][coordinate.second].cost <= collision_threshold); }
    inline int isOnTargetTrajectory(std::pair<int, int> coordinate) {
        auto anchor = target_trajectory.find(coordinate);
        if (anchor != target_trajectory.end())
            return anchor->second;
        else
            return -1;
    }

    Cell constructCell(std::pair<int, int> coordinate, int parent_x, int parent_y);
    void updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int cost);
    void addCellToOpenList(const std::pair<int, int> coordinate, const Cell& cell);
    void addCellToClosedList(const std::pair<int, int> coordinate, const Cell& cell);


    int manhattanDistance(std::pair<int, int> a, std::pair<int, int> b);
    std::pair<std::pair<int, int>, Cell> getCoordinateCellPairWithSmallestF();

    Cell getCellFromOpenList(std::pair<int, int> coordinate);

    bool isInOpenList(std::pair<int, int> coordinate);
    bool isInClosedList(std::pair<int, int> coordinate);
    bool hasBeenExplored(std::pair<int, int> coordinate);

    Path findPath(std::pair<int, int> coordinate);
    Path findBestPath(std::pair<int, int> coordinate, int curr_time);

 public:
    std::pair<int, int> start;
    std::pair<int, int> robot_start;
    std::pair<int, int> target_start;

    int collision_threshold;

    std::multimap<int, std::pair<int, int>> cost_to_coordinate;
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> open_list;
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> closed_list;

    std::unordered_map<std::pair<int, int>, int, pair_hash> target_trajectory;
    std::vector<std::pair<int, int>> ordered_target_trajectory;

    int total_num_cells;
    std::vector<std::vector<MapCell>> map;
    std::vector<std::vector<Cell>> cost_map;

    std::map<int, Path> evaluated_paths;

    bool is_initialized = false;
};
