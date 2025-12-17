#include "mini_snap/TrajectoryMake.hpp"

#include <iostream>
#include <Eigen/Dense>

int main() {
    std::cout<<"Hello world!"<<std::endl;

    std::vector<Eigen::Vector3d> pts = {
        {0, 0, 0},
        {1, 1, 1},
        {2, 0, 1}
    };
    std::vector<double> times = {2.0, 2.0};
    TrajectoryMake tra;
    tra.setWaypoints(pts);
    tra.setSegmentTimes(times);
    tra.solve(2);
    std::cout<<tra.get_coeff_x_()<<std::endl;
    return 0;
}