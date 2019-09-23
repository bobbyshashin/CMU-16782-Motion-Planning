#include <unordered_map>
#include <map>
#include <utility>
#include <vector>
#include <mex.h>

class AStarPlanner {

    struct Cell {
        int parent_x, parent_y;
        double F, G, H;

        Cell (int g, int h, int parent_x = -1, int parent_y = -1) : 
            G(g), H(h), parent_x(parent_x), parent_y(parent_y) {
                F = g + h;
        }
    };

    struct MapCell {
        int cost;
        bool is_initialized;
        MapCell() : cost(0), is_initialized(false) {}
        MapCell(int cost, bool is_initialized=false) : cost(cost), is_initialized(is_initialized) {}
    };

    // struct CoordinateFValuePair {
    //     std::pair<int, int> coordinate;
    //     int F;
    //     CoordinateCostPair(int x, int y, int F) : F(F) {
    //         coordinate = std::make_pair(x, y);
    //     }

    //     bool operator<(const CoordinateCostPair& p1, const CoordinateCostPair& p2) {
    //         return (p1.F < p2.F);
    //     } 
    // };

    // struct comp {
    //     bool operator() (const Cell& a, const Cell& b) const {
    //         return a.F <= b.F;
    //     }
    // };
    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

 public:
    // void search();

    void init(int start_x, int start_y, int goal_x, int goal_y, double weight=10.0);
    void initMapSize(int x_size, int y_size);
    void initParam(int collision_thres) {collision_threshold = collision_thres;}

    inline bool isInitialized() const { return is_initialized; }
    inline bool completed() const { return open_list.empty(); }

    inline bool isValidCoordinate(std::pair<int, int> coordinate) { return (coordinate.first >= 0 && coordinate.first < map.size() && coordinate.second >= 0 && coordinate.second < map[0].size() && map[coordinate.first][coordinate.second].cost <= collision_threshold); }

    void initMapCell(int x, int y, int cost);

    void updateCell(std::pair<int, int> coordinate, int parent_x, int parent_y, int G);
    void addCellToOpenList(const std::pair<int, int> coordinate, const Cell& cell);
    void addCellToClosedList(const std::pair<int, int> coordinate, const Cell& cell);

    
    int getHeuristic(int x, int y, double weight=10.0);
    std::pair<std::pair<int, int>, Cell> getCoordinateCellPairWithSmallestF();
    // std::pair<int, int> getCoordinateOfCellWithSmallestF();
    // Cell getCellWithSmallestF();
    Cell getCellFromOpenList(std::pair<int, int> coordinate);
    Cell constructCell(std::pair<int, int> coordinate, int parent_x, int parent_y);

    bool isInOpenList(std::pair<int, int> coordinate);
    bool isInClosedList(std::pair<int, int> coordinate);
    bool hasBeenExplored(std::pair<int, int> coordinate);

    std::pair<int, int> getNextAction(const std::pair<int, int>& goal);

 public:
    int start_x, start_y;
    int goal_x, goal_y;

    int collision_threshold;
    std::multimap<int, std::pair<int, int>> f_to_coordinate;
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> open_list;
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> closed_list;

    std::vector<std::vector<MapCell>> map;
    std::vector<std::vector<Cell>> cost_map;

    bool is_initialized = false;
};
