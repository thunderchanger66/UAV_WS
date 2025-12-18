#include "mini_snap/TrajectoryMake.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>
#include <iostream>

#include <fstream>

void TrajectoryMake::setWaypoints(const std::vector<Eigen::Vector3d>& pts) {
    waypoints_ = pts;
    if (pts.size() < 2) {
        // Need at least two waypoints to form a segment
        n_seg_ = 0;
        T_.clear();
        return;
    }
    n_seg_ = static_cast<int>(pts.size()) - 1; // 段数是端点数-1
    // resize T_ to hold times for each segment
    T_.assign(n_seg_, 0.0);
    //下面修改时间优化
    for (int i = 0; i < n_seg_; i++) {
        T_[i] = computeSegmentTime(pts[i], pts[i + 1]);
        T_[i] *= 1.2;//稍微放慢一点
        std::cout<<"T_["<<i<<"] = "<<T_[i]<<std::endl;
    }
}

//下面这个可以不要了
// void TrajectoryMake::setSegmentTimes(const std::vector<double>& times) {
//     //T_ = times;
//     n_seg_ = times.size();//段数就是时间的段数

// }

Eigen::MatrixXd TrajectoryMake::computeQ(double T) {
    Eigen::MatrixXd Q = Eigen::MatrixXd::Zero(n_coeff_, n_coeff_);
    for (int i = 4; i < n_coeff_; i++)
        for (int j = 4; j < n_coeff_; j++) {
            double ci = i * (i - 1) * (i - 2) * (i - 3);
            double cj = j * (j - 1) * (j - 2) * (j - 3);
            Q(i, j) = ci * cj * std::pow(T, i + j - 7) / (i + j - 7);
        }
    return Q;
}

void TrajectoryMake::buildCost() {
    int N = n_seg_ * n_coeff_;
    H_ = Eigen::MatrixXd::Zero(N, N);
    for (int i = 0; i < n_seg_; i++)
        H_.block(i * n_coeff_, i * n_coeff_, n_coeff_, n_coeff_) = computeQ(T_[i]);
}

//(i-0)(i-1)...(i-(d-1))*t^(i-d)就是第d阶导数
Eigen::VectorXd TrajectoryMake::derivativeBasis(double t, int d) {
    Eigen::VectorXd basis = Eigen::VectorXd::Zero(n_coeff_);
    for (int i = d; i < n_coeff_; i++) {
        double coeff_ = 1.0;
        for (int j = 0; j < d; j++) {
            coeff_ *= i - j;
        }
        basis(i) = coeff_ * std::pow(t, i - d);
    }
    return basis;
}

void TrajectoryMake::buildConstraints(int xyz) {
    //1.途径点位置约束 2.边界条件约束 3.连续性约束
    //起点/终点各4个（位置，速度，加速度，jerk）
    //中间点（waypoints-2）*4个（去掉起点终点，包含连续性），waypoints=n_seg_+1
    //现在还要加上路径点到达的约束n_seg_-1个，也就是waypoints-2
    int n_constraints = 4 * 2 + (waypoints_.size() - 2) * 4 + waypoints_.size() - 2;
    //rows=约束数量，cols=每段轨迹系数数量(8)*总段数
    A_ = Eigen::MatrixXd::Zero(n_constraints, n_coeff_ * n_seg_);
    //rows=约束数量
    b_ = Eigen::VectorXd::Zero(n_constraints);

    int row = 0;
    //1.起点约束:给定起始位置，但是v，a，j都是0，t=0时刻
    for (int i = 0; i < 4; i++) {
        Eigen::VectorXd basis = derivativeBasis(0.0, i);
        A_.block(row, 0, 1, n_coeff_) = basis.transpose();
        //注意这里确定了是x，y，z方向上的哪个
        b_(row) = (i == 0) ? waypoints_.front()(xyz) : 0.0;
        row++;
    }

    //2.中间点的位置与连续性约束,从第（2-1）个点开始，到n_seg_-1结束
    //这里是位置约束，用的是前一段的轨迹系数，所以需要（i-1），同时T_[i - 1]才是这一段时间
    // 中间 waypoint 的位置约束（只约束前一段的末端）
    for (int i = 1; i < n_seg_; i++) {
        int col = (i - 1) * n_coeff_;
        Eigen::VectorXd basis = derivativeBasis(T_[i - 1], 0);
        A_.block(row, col, 1, n_coeff_) = basis.transpose();
        b_(row) = waypoints_.at(i)(xyz);
        row++;
    }
    //然后是连续性约束
    //注意，位置连续性从这里开始，j从0开始，包含位置，速度，加速度以及jerk
    for (int i = 1; i < n_seg_; i++)
        for (int j = 0; j < 4; j++) {
            int col_prev = (i - 1) * n_coeff_;
            int col_next = i * n_coeff_;
            Eigen::VectorXd basis_prev = derivativeBasis(T_[i - 1], j);
            Eigen::VectorXd basis_next = derivativeBasis(0.0, j);
            A_.block(row, col_prev, 1, n_coeff_) = basis_prev.transpose();
            A_.block(row, col_next, 1, n_coeff_) = -basis_next.transpose();
            b_(row) = 0.0;
            row++;
        }

    //3.终点约束
    //最后列的开始为（段数-1）乘以轨迹系数的个数
    int last_col = (n_seg_ - 1) * n_coeff_;
    double T_end = T_.back();//最后一个时间
    for (int i = 0; i < 4; i++) {
        Eigen::VectorXd basis = derivativeBasis(T_end, i);
        A_.block(row, last_col, 1, n_coeff_) = basis.transpose();
        b_(row) = (i == 0) ? waypoints_.back()(xyz) : 0.0;
        row++;
    }
}

void TrajectoryMake::solve() {
    for (int i = 0; i < 3; i++) {//分别求解xyz方向上的
        buildCost();
        buildConstraints(i);
        Eigen::MatrixXd KKT(H_.rows() + A_.rows(), H_.cols() + A_.rows());//这里列数A转置了，所以直接用行
        KKT << H_, A_.transpose(), A_, Eigen::MatrixXd::Zero(A_.rows(), A_.rows());
        Eigen::VectorXd rhs(H_.rows() + A_.rows());
        rhs << Eigen::VectorXd::Zero(H_.rows()), b_;
        Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs);
        switch(i) {//确定每个方向的系数
            case 0:
                coeff_x_ = sol.head(H_.rows());
                break;
            case 1:
                coeff_y_ = sol.head(H_.rows());
                break;
            case 2:
                coeff_z_ = sol.head(H_.rows());
        }
    }
}

//采用最大加速-最大匀速-最大减速的时间段寻找方式，优化时间
//但是要注意是否能到匀速，可能加减速就走完整段路程
//这样就分为梯形位移图及三角形位移图
// 加速v^2/(2ax),减速同理，则加减速总路程乘2
double TrajectoryMake::computeSegmentTime(
    const Eigen::Vector3d& p0,
    const Eigen::Vector3d& p1) {
        double d = (p1 - p0).norm();
        double d_min = v_max * v_max / a_max;
        if (d > d_min) {
            double t_acc = v_max / a_max;
            double d_steady = d - d_min;
            double t_steady = d_steady / v_max;
            return t_acc * 2 + t_steady;
        }
        else//这里一段是1/2a\Delta{t}，两段加一起是d，但是这是\Delta{t}，总的t是乘2
            return std::sqrt(d / a_max) * 2;
    }

//这里设定每100ms采样1次，与offboard控制模式的控制频率相同
void TrajectoryMake::sample(double dt) {
    bool first_seg = true;
    for (int i = 0; i < n_seg_; i++) {//总段数
        int steps = static_cast<int>(std::floor(T_[i] / dt));
        for (int j = 0; j <= steps; j++) {//每100ms采样1次，同时在第i段
            if (!first_seg && j == 0) continue;// 如果不是首段，那么下一段起始值和前一段最后的值相等，不用再算
            double t = std::min(j * dt, T_[i]);

            double x = 0.0, y = 0.0, z = 0.0;
            for (int k = 0; k < n_coeff_; k++) {//多项式求和
                double tk = std::pow(t, k);
                x += coeff_x_[i * n_coeff_ + k] * tk;
                y += coeff_y_[i * n_coeff_ + k] * tk;
                z += coeff_z_[i * n_coeff_ + k] * tk;
            }
            sample_x_.push_back(x);
            sample_y_.push_back(y);
            sample_z_.push_back(z);
            count++;
        }
        first_seg = false;
    }
}