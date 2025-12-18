#pragma once

#include <Eigen/Dense>
#include <vector>

/*--------------
1.首先确定使用7阶多项式
2.x，y，z以及\psi都要单独规划，这里先实验x的
3.先计算Q_i，构造H矩阵
4.再处理约束，A矩阵和b矩阵
5.使用KKT求解
--------------*/

class TrajectoryMake {
public:
    //假设7阶多项式，最大速度和加速度都是2
    TrajectoryMake(int order = 7, double vmax = 2, double amax = 2): 
        order_(order), n_coeff_(order + 1), v_max(vmax), a_max(amax) {}

    void setWaypoints(const std::vector<Eigen::Vector3d>& pts);
    //void setSegmentTimes(const std::vector<double>& times);

    void solve(int xyz);

    Eigen::VectorXd get_coeff_x_() { return coeff_x_; }
    Eigen::VectorXd get_coeff_y_() { return coeff_y_; }
    Eigen::VectorXd get_coeff_z_() { return coeff_z_; }
    //前n_coeff_ * n_seg_是x，以此类推
    Eigen::VectorXd get_coeff_xyz_() {
        Eigen::VectorXd coeff_xyz_(n_coeff_ * n_seg_ * 3);
        coeff_xyz_ << coeff_x_, coeff_y_, coeff_z_;
        return coeff_xyz_;
    }

    double computeSegmentTime(
        const Eigen::Vector3d& p0,
        const Eigen::Vector3d& p1);

private:
    int order_;//阶数
    int n_coeff_;//系数的数目，等于order+1
    int n_seg_;//分段的数目，跟waypoints的数目有关，Waypoints-1

    double v_max, a_max;//事先定义好最大速度及最大加速度

    std::vector<double> T_;//每段的时间
    std::vector<Eigen::Vector3d> waypoints_;//途径点的集合

    Eigen::MatrixXd H_;//H矩阵
    Eigen::MatrixXd A_;//A矩阵
    Eigen::VectorXd b_;//b向量
    Eigen::VectorXd coeff_x_, coeff_y_, coeff_z_;//x，y，z每个方向上的系数向量

    Eigen::MatrixXd computeQ(double T);
    void buildCost();

    //需要计算在t时刻，第d阶导数的系数向量
    Eigen::VectorXd derivativeBasis(double t, int d);
    //这里要给定是计算哪个方向的约束，定义0，1，2为x，y，z
    void buildConstraints(int xyz);
};