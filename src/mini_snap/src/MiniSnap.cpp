#include "mini_snap/TrajectoryMake.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <fstream>

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
    tra.solve();
    tra.sample();

    // 打开文件
    std::ofstream file("trajectory_samples.csv");//
    if (!file.is_open()) {
        std::cerr << "无法打开文件!" << std::endl;
        return 0;
    }//
    // 写入CSV头部
    file << "x,y,z\n";//
    // 写入CSV文件
    for (int i = 0; i < tra.get_count_(); i++) {
        file << tra.get_x_()[i] << "," << tra.get_y_()[i] << ","
            << tra.get_z_()[i] << "\n";//
    }
    file.close();//
    std::cout << "采样数据已保存到 trajectory_samples.csv" << std::endl;//

    return 0;
}