#include <unordered_map>
#include <map>
#include <queue>


class Dijkstra {
 public:
    struct Cell {
        int x, y;
        int parent_x, parent_y;
        int cost;
        int step_distance;

        Cell (int x, int y, int cost, int parent_x = -1, int parent_y = -1, int step_distance = 0) : 
            x(x), y(y), cost(cost), parent_x(parent_x), parent_y(parent_y), step_distance(step_distance) {}
    };

    struct Path {
        std::vector<std::pair<int, int>> trajectory;
        int length;
        int cost;

        int least_cost_index;
        std::pair<int, int> least_cost_coordinate;
        int wait_time;
    };

    struct compareCost {
        bool operator()(const Cell& lhs, const Cell& rhs) const { return lhs.cost > rhs.cost; }
    };

    struct pair_hash {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    void initMapSize(int x, int y);
    void init(int start_x, int start_y, int collision_threshold);
    void initTargetTrajectory(std::vector<std::pair<int, int>> traj);

    void search();

    void addToOpenList(const Cell cell);
    void addToClosedList(const Cell cell);

    bool isValid(int x, int y);
    inline bool isInOpenList(int x, int y) const { return visited[x][y]; }
    inline bool isInClosedList(int x, int y) const { return explored[x][y]; }

    inline bool isInitialized() const {return is_initialized; }
    inline bool isCompleted() const { return open_list.empty(); }

    Cell extractFromOpenList();

    Path findPath(int x, int y);
    void optimizePath(Path& path);

    int start_x, start_y;

    int map_size_x, map_size_y;

    int collision_threshold;
    bool is_initialized = false;

    // pass this from init later
    int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

    std::vector<std::vector<int>> map;
    std::vector<std::vector<int>> cost_map;
    std::vector<std::vector<int>> parents_x;
    std::vector<std::vector<int>> parents_y;
    std::vector<std::vector<bool>> visited; // has been added to open list
    std::vector<std::vector<bool>> explored; // has been added to closed list

    std::priority_queue<Cell, std::vector<Cell>, compareCost> open_list;
    std::unordered_map<std::pair<int, int>, Cell, pair_hash> closed_list;

    std::unordered_multimap<std::pair<int, int>, int, pair_hash> target_trajectory;
    std::map<int, Path> evaluated_paths;

 private:
    void generatePath();

};
