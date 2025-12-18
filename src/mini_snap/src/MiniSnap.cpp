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
    //std::vector<double> times = {2.0, 2.0};
    TrajectoryMake tra;
    tra.setWaypoints(pts);
    for (int i = 0; i < 3; i++)
        tra.solve(i);
    
    //std::cout<<tra.get_coeff_xyz_().transpose()<<std::endl;
    std::cout<<tra.get_coeff_x_().transpose()<<std::endl;
    std::cout<<tra.get_coeff_y_().transpose()<<std::endl;
    std::cout<<tra.get_coeff_z_().transpose()<<std::endl;
    return 0;
}