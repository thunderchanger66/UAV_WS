#include "Astar/Astar.hpp"

#include <utility>
#include <queue>
#include <memory>
#include <algorithm>
#include <unordered_map>

std::vector<Astar::Node*> Astar::getNeighbors(Node* node, bool use_diagonal, const std::vector<std::vector<int>>& grid) {
    static const int dx4[4] = {1, -1, 0, 0};
    static const int dy4[4] = {0, 0, 1, -1};
    
    static const int dx8[8] = {1, 1, 0, -1, -1, -1, 0, 1};
    static const int dy8[8] = {0, 1, 1, 1, 0, -1, -1, -1};

    std::vector<Node*> neighbors;

    int count = use_diagonal ? 8 : 4;
    const int* dx = use_diagonal ? dx8 : dx4;
    const int* dy = use_diagonal ? dy8 : dy4;

    for(int i = 0; i < count; i++) {
        int nx = node->x + dx[i];
        int ny = node->y + dy[i];
        //这里考虑是否越界及是否为障碍物
        if(inbounds(nx, ny, rows, cols) && !grid[nx][ny]) {
            neighbors.push_back(new Node(nx, ny));
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

        //用于管理所有节点的声明周期
        std::unordered_map<int, Node*> all_nodes;
        
        auto start_node = std::make_unique<Node>(Node(start.first, start.second));
        start_node->h = heuristic(start, goal);
        start_node->f = start_node->h;
        open.push(start_node.get());
        all_nodes[hash(start.first, start.second)] = start_node.get();

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
            closed[x][y] = true;//标记为已访问
            auto neighbors = getNeighbors(current, use_diagonal, grid);
            for(auto& neighbor : neighbors) {
                if(closed[neighbor->x][neighbor->y]) continue;

                float step_cost = (neighbor->x != x && neighbor->y != y) ? 1.414f : 1.0f;
                float tentative_g = current->g + step_cost;

                int key = hash(neighbor->x, neighbor->y);
                if(!all_nodes.count(key) || tentative_g < all_nodes[key]->g) {//未访问过或者此时g值更小
                    neighbor->parent = current;
                    neighbor->g = tentative_g;
                    neighbor->h = heuristic({neighbor->x, neighbor->y}, goal);
                    neighbor->f = neighbor->g + neighbor->h;

                    open.push(neighbor);
                    all_nodes[key] = neighbor;
                }
            }
        }
        return {};
}