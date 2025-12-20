#pragma once

#include "mini_snap/TrajectoryMake.hpp"

#include <iostream>
#include <Eigen/Dense>
#include <vector>
#include <cmath>

#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_odometry.hpp"

/*待优化
1.命令行坐标输入 //完成，传入整个数组，在每三个解析即可
2.完全arm后再启动 //不用管这个
3.Rviz中显示轨迹
*/
//首先我要将当前位置作为起点，而不是随意给定
class ToFly : public rclcpp::Node {
public:
    ToFly() : Node("tofly") {
        //参数声明，xyz点
        this->declare_parameter<std::vector<double>>("points", std::vector<double>{});//加参数 -p points:="[x0,y0,z0, x1,y1,z1...]"
        //获取参数
        this->get_parameter("points", points);
        if (points.size() % 3 != 0) {
            RCLCPP_ERROR(this->get_logger(), "Parameter fault!");
            return;
        }

        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>(
            "/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>(
            "/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_odometry_subscription_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "/fmu/out/vehicle_odometry", qos,
            std::bind(&ToFly::vehicle_odometry_subscription_callback, this, std::placeholders::_1)
        );
        got_initial_pose_ = false;

        offboard_setpoint_counter_ = 0;
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            if(got_initial_pose_) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }

            if (offboard_setpoint_counter_ < 11) offboard_setpoint_counter_++;
        };
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), timer_callback);
    }

    void arm();

private:
    rclcpp::TimerBase::SharedPtr timer_;

    rclcpp::Publisher<px4_msgs::msg::OffboardControlMode>::SharedPtr offboard_control_mode_publisher_;
    rclcpp::Publisher<px4_msgs::msg::TrajectorySetpoint>::SharedPtr trajectory_setpoint_publisher_;
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr vehicle_command_publisher_;

    uint64_t offboard_setpoint_counter_;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

    int traj_index = 0;//确定现在跟踪哪个点

    //真正的离散的路径点
    std::vector<double> x_pos_, y_pos_, z_pos_;

    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_subscription_;
    void vehicle_odometry_subscription_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);
    bool got_initial_pose_;//是否是第一次进入当前起始位置 第一次收到 odometry，用它作为起点
    TrajectoryMake tra_;
    void planTrajectory(const std::vector<Eigen::Vector3d>& pts);

    //更方便的传入途径点    //创建途径点集合
    std::vector<double> points;
};