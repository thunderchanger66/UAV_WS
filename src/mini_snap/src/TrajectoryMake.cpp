#include "mini_snap/TrajectoryMake.hpp"

#include <Eigen/Dense>
#include <vector>
#include <cmath>

void TrajectoryMake::setWaypoints(const std::vector<Eigen::Vector3d>& pts) {
    waypoints_ = pts;
}

void TrajectoryMake::setSegmentTimes(const std::vector<double>& times) {
    T_ = times;
    n_seg_ = times.size();//段数就是时间的段数
}

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

void TrajectoryMake::solve(int xyz) {
    buildCost();
    buildConstraints(xyz);
    Eigen::MatrixXd KKT(H_.rows() + A_.rows(), H_.cols() + A_.rows());//这里列数A转置了，所以直接用行
    KKT << H_, A_.transpose(), A_, Eigen::MatrixXd::Zero(A_.rows(), A_.rows());
    Eigen::VectorXd rhs(H_.rows() + A_.rows());
    rhs << Eigen::VectorXd::Zero(H_.rows()), b_;
    Eigen::VectorXd sol = KKT.fullPivLu().solve(rhs);
    coeff_x_ = sol.head(H_.rows()); 
}