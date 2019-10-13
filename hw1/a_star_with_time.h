#include <unordered_map>
#include <map>
#include <utility>
#include <vector>
#include <mex.h>

class AStarPlanner {

    struct Cell {
        int parent_x, parent_y;
        double F, G, H;
        int T;

        Cell (int g, int h, int t, int parent_x = -1, int parent_y = -1) : 
            G(g), H(h), T(t), parent_x(parent_x), parent_y(parent_y) {
                F = g + h;
        }
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

    struct Path {
        std::vector<std::pair<int, int>> path;
        int length;
        int position_on_target_traj;
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    struct tuple_hash {
        template <class T1, class T2, class T3>
        std::size_t operator() (const std::pair<T1, T2, T3> &tuple) const {
            return std::hash<T1>()(std::get<0>(tuple)) ^ std::hash<T2>()(std::get<1>(tuple)) ^ std::hash<T3>()(std::get<2>(tuple));
        }
    };

 public:
    // void search();

    void init(int start_x, int start_y, int goal_x, int goal_y, double weight=1.0);
    void initMapSize(int x_size, int y_size);
    void initParam(std::pair<int, int> robot_start, std::pair<int, int> target_start, int collision_thres) {
        this->robot_start = robot_start;
        this->target_start = target_start;
        collision_threshold = collision_thres;
    }

    void initTargetTrajectory(std::vector<std::pair<int, int>> traj);

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

    void initMapCell(std::pair<int, int> coordinate, int cost);

    void updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int G);
    void addCellToOpenList(const std::pair<int, int> coordinate, int time, const Cell& cell);
    void addCellToClosedList(const std::pair<int, int> coordinate, int time, const Cell& cell);

    
    int getHeuristic(int x, int y);
    int manhattanDistance(std::pair<int, int> a, std::pair<int, int> b);
    std::pair<std::pair<int, int>, Cell> getCoordinateCellPairWithSmallestF();

    Cell getCellFromOpenList(std::pair<int, int> coordinate);
    Cell constructCell(std::pair<int, int> coordinate, int time, int parent_x, int parent_y);

    bool isInOpenList(std::pair<int, int> coordinate);
    bool isInClosedList(std::pair<int, int> coordinate);
    bool hasBeenExplored(std::pair<int, int> coordinate);

    std::pair<int, int> getNextAction(const std::pair<int, int>& goal);

    Path findPath(std::pair<int, int> coordinate);

 public:

    std::pair<int, int> robot_start;
    std::pair<int, int> target_start;

    int start_x, start_y;
    int goal_x, goal_y;

    int collision_threshold;
    int heuristic_weight;
    std::multimap<int, std::tuple<int, int, int>> f_to_coordinate;
    std::unordered_map<std::tuple<int, int, int>, Cell, pair_hash> open_list;
    std::unordered_map<std::tuple<int, int, int>, Cell, pair_hash> closed_list;
    std::unordered_map<std::pair<int, int>, int, pair_hash> heuristics;

    std::unordered_map<std::pair<int, int>, bool, pair_hash> target_trajectory;

    std::map<int, std::pair<int, int>> best_goal;

    std::vector<std::vector<MapCell>> map;

    

    bool is_initialized = false;
};
