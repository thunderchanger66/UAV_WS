#include "mini_snap/ToFly.hpp"

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
	msg.position = {(float)x_pos_[idx], -(float)y_pos_[idx], -(float)z_pos_[idx]};
    traj_index++;
	msg.yaw = std::atan2(y_pos_[idx], x_pos_[idx]); // [-PI:PI]
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	trajectory_setpoint_publisher_->publish(msg);
    //RCLCPP_INFO(this->get_logger(), "test\n");
}

void ToFly::vehicle_odometry_subscription_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
    if (!got_initial_pose_) {//接收到了起始位置
        double x0 = msg->position[0];
        double y0 = -msg->position[1];
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