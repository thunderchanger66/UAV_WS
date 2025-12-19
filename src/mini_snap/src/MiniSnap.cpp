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
2.完全arm后再启动
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
        offboard_flag = false;

        offboard_setpoint_counter_ = 0;
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
                offboard_flag = true;//这里注意，需要确定无人机已经完全arm才能发送坐标命令，需要优化
            }

            if(got_initial_pose_ && offboard_flag) {//需要获得初始位置及已经在offboard模式
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

    bool offboard_flag;//是否进入offboard模式

    //更方便的传入途径点    //创建途径点集合
    std::vector<double> points;
};

void ToFly::publish_vehicle_command(uint16_t command, float param1, float param2) {
    px4_msgs::msg::VehicleCommand msg{};
    msg.param1 = param1;
    msg.param2 = param2;
    msg.command = command;
    msg.target_system = 1;
    msg.target_component = 1;
    msg.source_system = 1;
    msg.source_component = 1;
    msg.from_external = true;
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    vehicle_command_publisher_->publish(msg);
}

void ToFly::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void ToFly::publish_trajectory_setpoint()
{
    if (x_pos_.empty() || y_pos_.empty() || z_pos_.empty()) {
        RCLCPP_WARN(this->get_logger(), "Trajectory empty, skipping publish");
        return;
    }
	px4_msgs::msg::TrajectorySetpoint msg{};
    int idx = std::min(traj_index, (int)x_pos_.size() - 1);
    //!!!!!!忘记了ROS与PX4的坐标表示不一样，Z轴反过来的，一开始飞机不动，是Z轴没加负号
	msg.position = {(float)x_pos_[idx], (float)y_pos_[idx], -(float)z_pos_[idx]};
    traj_index++;
	msg.yaw = std::atan2(y_pos_[idx], x_pos_[idx]); // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
    RCLCPP_INFO(this->get_logger(), "test\n");
}

void ToFly::vehicle_odometry_subscription_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
    if (!got_initial_pose_) {//接收到了起始位置
        double x0 = msg->position[0];
        double y0 = msg->position[1];
        double z0 = -msg->position[2];

        std::vector<Eigen::Vector3d> pts;
        pts.emplace_back(x0, y0, z0);

        for (size_t i = 0; i < points.size(); i+=3) 
            pts.emplace_back(points[i], points[i + 1], points[i + 2]);

        planTrajectory(pts);
        RCLCPP_INFO(this->get_logger(), "Got initial pose and planned trajectory: start=(%f, %f, %f), samples=%zu", x0, y0, z0, x_pos_.size());

        got_initial_pose_ = true;
    }
}

void ToFly::planTrajectory(const std::vector<Eigen::Vector3d>& pts) {
    tra_.setWaypoints(pts);
    tra_.solve();
    tra_.sample();

    x_pos_ = tra_.get_x_();
    y_pos_ = tra_.get_y_();
    z_pos_ = tra_.get_z_();
}

void ToFly::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

int main(int argc, char *argv[]) {
    std::cout<<"Hello world!"<<std::endl;
    std::cout << "Starting offboard control mode..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToFly>());

    rclcpp::shutdown();

    // std::vector<Eigen::Vector3d> pts = {
    //     {0, 0, 0},
    //     {1, 1, 1},
    //     {2, 0, 1}
    // };
    // //std::vector<double> times = {2.0, 2.0};
    // TrajectoryMake tra;
    // tra.setWaypoints(pts);
    // tra.solve();
    // tra.sample();

    return 0;
}