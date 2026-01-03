#include "astar/Astar.hpp"

#include <utility>
#include <queue>
#include <memory>
#include <algorithm>
#include <unordered_map>

std::vector<std::pair<int,int>> Astar::getNeighbors(Node* node, bool use_diagonal, const std::vector<std::vector<int>>& grid) {
    static const int dx4[4] = {1, -1, 0, 0};
    static const int dy4[4] = {0, 0, 1, -1};
    
    static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    std::vector<std::pair<int,int>> neighbors;

    int count = use_diagonal ? 8 : 4;
    const int* dx = use_diagonal ? dx8 : dx4;
    const int* dy = use_diagonal ? dy8 : dy4;

    for(int i = 0; i < count; i++) {
        int nx = node->x + dx[i];
        int ny = node->y + dy[i];
        //这里考虑是否越界及是否为障碍物
        if(inbounds(nx, ny) && !grid[nx][ny]) {
            neighbors.emplace_back(nx, ny);
        }
    }
    return neighbors;
}

std::vector<std::pair<int, int>> Astar::findPath(const std::vector<std::vector<int>>& grid, 
    const std::pair<int, int>& start, const std::pair<int, int>& goal, bool use_diagonal) {
        auto cmp = [](Node* a, Node* b) {
            return a->f > b->f;
        };
        std::priority_queue<Node*, std::vector<Node*>, decltype(cmp)> open(cmp);//小顶堆，开放列表
        rows = grid.size();
        cols = grid[0].size();
        std::vector<std::vector<bool>> closed(rows, std::vector<bool>(cols, false));//关闭列表，开始没有元素
        
        auto hash = [&](int x, int y) {
            return y * cols + x;
        };

        //边界/障碍检查
        if(!inbounds(start.first, start.second) || !inbounds(goal.first, goal.second)) return {};
        if(grid[start.second][start.first] != 0 || grid[goal.second][goal.first] != 0) return {};

        //用于管理所有节点的声明周期（unique_ptr），其实就是为了更新更好的g值
        std::unordered_map<int, std::unique_ptr<Node>> all_nodes;
        
        int start_key = hash(start.first, start.second);
        all_nodes[start_key] = std::make_unique<Node>(start.first, start.second);
        Node* start_ptr = all_nodes[start_key].get();
        start_ptr->g = 0.0f;
        start_ptr->h = heuristic(start, goal);
        start_ptr->f = start_ptr->h;
        open.push(start_ptr);

        while (!open.empty()) {
            Node* current = open.top();
            open.pop();
            int x = current->x, y = current->y;
            //如果已经到达终点
            if(x == goal.first && y == goal.second) {
                std::vector<std::pair<int, int>> path;
                while(current) {
                    path.emplace_back(current->x, current->y);
                    current = current->parent;
                }
                std::reverse(path.begin(), path.end());
                return path;
            }
            // 跳过 stale 条目（priority_queue 没有 decrease-key）：如果当前弹出的节点
            // 不是 all_nodes 中该坐标的最新节点，就跳过它
            int current_key = hash(x, y);
            if (!all_nodes.count(current_key) || current != all_nodes[current_key].get()) continue;

            closed[x][y] = true;//标记为已访问（注意索引顺序为 closed[row][col] = closed[y][x])
            auto neighbors = getNeighbors(current, use_diagonal, grid);
            for(auto& coord : neighbors) {
                int nx = coord.first, ny = coord.second;
                if(closed[nx][ny]) continue;

                // 可选：禁止对角穿角（防止穿过被两个障碍夹住的角）
                // if (nx != x && ny != y) {
                //     if (grid[nx][y] != 0 || grid[x][ny] != 0) continue;
                // }

                float step_cost = (nx != x && ny != y) ? 1.414f : 1.0f;
                float tentative_g = current->g + step_cost;

                int key = hash(nx, ny);
                if(!all_nodes.count(key)) {
                    all_nodes[key] = std::make_unique<Node>(nx, ny);
                }
                Node* neighbor = all_nodes[key].get();

                if(tentative_g < neighbor->g) {//未访问过或者此时g值更小
                    neighbor->parent = current;
                    neighbor->g = tentative_g;
                    neighbor->h = heuristic({nx, ny}, goal);
                    neighbor->f = neighbor->g + neighbor->h;

                    open.push(neighbor);
                }
            }
        }
        return {};
}