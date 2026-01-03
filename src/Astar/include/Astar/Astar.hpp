#pragma once

#include <utility>
#include <vector>
#include <cmath>

class Astar {
public:
    //grid==0无障碍，grid==1有障碍
    std::vector<std::pair<int, int>> findPath(const std::vector<std::vector<int>>& grid, 
        const std::pair<int, int>& start, const std::pair<int, int>& goal, bool use_diagonal = true);

private:
    struct Node {
        int x, y;//当前节点坐标
        float f, g, h;//代价函数值
        Node* parent;//父节点，回溯用
        Node(int x_, int y_) : x(x_), y(y_), f(0), g(0), h(0), parent(nullptr) {}
    };

    float heuristic(std::pair<int, int> a, std::pair<int, int> b) {//h函数，欧几里得距离
        return std::hypot(a.first - b.first, a.second - b.second);
    }

    //获得邻居
    std::vector<Node*> getNeighbors(Node* node, bool use_diagonal, const std::vector<std::vector<int>>& grid);
    bool inbounds(int x, int y, int rows, int cols) const {
        return x >= 0 && x < rows && y >= 0 && y < cols;
    }

    int rows, cols;//地图行列大小
};