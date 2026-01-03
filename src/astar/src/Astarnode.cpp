#include "astar/Astar.hpp"

#include <iostream>

int main() {
    Astar astar;

    std::vector<std::vector<int>> grid{{0, 0, 0, 0, 1},
                                       {0, 1, 1, 1, 0},
                                       {0, 0, 0, 1, 0},
                                       {0, 0, 1, 1, 0},
                                       {0, 0, 0, 0, 0}};
    std::vector<std::pair<int, int>> path = astar.findPath(grid, {0, 0}, {4, 4});
    for(auto& point : path) {
        std::cout<<"("<<point.first<<", "<<point.second<<")"<<std::endl;
    }
    std::cout<<"Hello World!"<<std::endl; 

    return 0;
}