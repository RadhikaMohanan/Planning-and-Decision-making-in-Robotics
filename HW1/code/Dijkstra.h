#include <iostream>
#include <queue>
#include <map>
#include <unordered_map>


class Dijkstra {
 public:
    struct elem {
        int x, y;
        int parent_x, parent_y;
        int cost;
        int step_distance;

        elem (int x, int y, int cost, int parent_x = -1, int parent_y = -1, int step_distance = 0) :
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

    struct costcompare {
        bool operator()(const elem& lhs, const elem& rhs) const { return lhs.cost > rhs.cost; }
    };

    struct pair_val {
        template <class T1, class T2>
        std::size_t operator() (const std::pair<T1, T2> &pair) const {
            return std::hash<T1>()(pair.first) ^ std::hash<T2>()(pair.second);
        }
    };

    void initmap(int x_size, int y_size) {
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

    void init(int start_x, int start_y, int collision_threshold){

    this->start_x = start_x;
    this->start_y = start_y;
    this->collision_threshold = collision_threshold;

    elem start(start_x, start_y, map[start_x][start_y]);
    add_open_list(start);
}
    void init_target_traj(std::vector<std::pair<int, int>> traj) {
    for (int i=0; i<traj.size(); ++i) {
        target_trajectory.insert({traj[i], i});
    }
}


    void search(){

    while (!isCompleted()) {


        const auto& curr_cell = extractFromOpenList();
        add_closed_list(curr_cell);
        int curr_x = curr_cell.x;
        int curr_y = curr_cell.y;

        for (int i=0; i<8; ++i) {
            int new_x = curr_x + dX[i];
            int new_y = curr_y + dY[i];

            if (isValid(new_x, new_y) && !isInClosedList(new_x, new_y)) {
                int new_cost = cost_map[curr_x][curr_y] + map[new_x][new_y];
                const bool lower_cost = new_cost < cost_map[new_x][new_y];

                if (!isInOpenList(new_x, new_y) || lower_cost) {
                    elem new_cell(new_x, new_y, new_cost, curr_x, curr_y, curr_cell.step_distance + 1);
                    add_open_list(new_cell);

                }
            }
        }
    }

    generatePath();
}
        void generatePath(){

    for (const auto& point : target_trajectory) {

        int x = point.first.first;
        int y = point.first.second;
        int target_arrival_time = point.second;

        if (!isInClosedList(x, y)) {
            continue;
        }

        auto path = pathfind(x, y);
        if (path.cost == -1) {
            continue;
        }
        int wait_time = target_arrival_time - path.length;
        if (wait_time > 1) {

            path.wait_time = wait_time;
            optimizePath(path);
            evaluated_paths.insert({path.cost + wait_time * map[path.least_cost_coordinate.first][path.least_cost_coordinate.second], path});
        }

    }

}
    void add_open_list(const elem cell){
    open_list.push(cell);

    cost_map[cell.x][cell.y] = cell.cost;
    visited[cell.x][cell.y] = true;
    parents_x[cell.x][cell.y] = cell.parent_x;
    parents_y[cell.x][cell.y] = cell.parent_y;
}
    void add_closed_list(const elem cell){
    explored[cell.x][cell.y] = true;


}

    bool isValid(int x, int y){

    return (x >= 0 &&
            y >= 0 &&
            x < map_size_x &&
            y < map_size_y &&
            map[x][y] < collision_threshold);
}
    inline bool isInOpenList(int x, int y) const { return visited[x][y]; }
    inline bool isInClosedList(int x, int y) const { return explored[x][y]; }

    inline bool isInitialized() const {return is_initialized; }
    inline bool isCompleted() const { return open_list.empty(); }

    elem extractFromOpenList(){

    if (!open_list.empty()) {
        const auto& c = open_list.top();
        elem cell(c.x, c.y, c.cost, c.parent_x, c.parent_y, c.step_distance);
        open_list.pop();
        return cell;
    }

}

    Path pathfind(int x0, int y0){
    Path path;
    int x = x0;
    int y = y0;

    path.cost = cost_map[x][y];
    path.least_cost_coordinate = std::make_pair(x, y);

    int index = 0;
    path.least_cost_index = index;
    int least_cost = map[x][y];

    path.trajectory.push_back(std::make_pair(x, y));

    while (x != start_x || y != start_y) {
        if (!isInClosedList(x, y)) {
            mexPrintf("Unable to find path from Dijkstra! \n");
            path.cost = -1;
            return path;
        }
        int px = parents_x[x][y];
        int py = parents_y[x][y];
        x = px;
        y = py;

        index++;
        if (isValid(x, y)) {
            if (map[x][y] < least_cost) {
                path.least_cost_coordinate.first = x;
                path.least_cost_coordinate.second = y;
                least_cost = map[x][y];
                path.least_cost_index = index;
            }
            path.trajectory.push_back(std::make_pair(x, y));
        }
        else {
        }

    }

    path.length = path.trajectory.size();

    return path;
}
    void optimizePath(Path& path){

    auto coordinate = path.least_cost_coordinate;
    auto itr = path.trajectory.insert(path.trajectory.begin() + path.least_cost_index, path.wait_time, coordinate);
}

    int start_x, start_y;

    int map_size_x, map_size_y;

    int collision_threshold;
    bool is_initialized = false;

    int dX[8] = {-1, -1, -1,  0,  0,  1, 1, 1};
    int dY[8] = {-1,  0,  1, -1,  1, -1, 0, 1};

    std::vector<std::vector<int>> map;
    std::vector<std::vector<int>> cost_map;
    std::vector<std::vector<int>> parents_x;
    std::vector<std::vector<int>> parents_y;
    std::vector<std::vector<bool>> visited;
    std::vector<std::vector<bool>> explored;

    std::priority_queue<elem, std::vector<elem>, costcompare> open_list;
    std::unordered_map<std::pair<int, int>, elem, pair_val> closed_list;

    std::unordered_multimap<std::pair<int, int>, int, pair_val> target_trajectory;
    std::map<int, Path> evaluated_paths;



};
