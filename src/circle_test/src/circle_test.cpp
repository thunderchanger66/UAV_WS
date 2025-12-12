#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_attitude.hpp"

#include <chrono>
#include <iostream>
#include <atomic>

#include <Eigen/Dense>

class CircleTest : public rclcpp::Node {
public:
    CircleTest() : Node("circle_test") {
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
		auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_attitude_subscription_ = this->create_subscription<px4_msgs::msg::VehicleAttitude>(
            "/fmu/out/vehicle_attitude", qos,
            std::bind(&CircleTest::vehicle_attitude_subscription_callback, this, std::placeholders::_1)
        );

        offboard_setpoint_counter_ = 0;

        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            publish_offboard_control_mode();
            publish_trajectory_setpoint();

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

    std::atomic<uint64_t> timestamp_;

    uint64_t offboard_setpoint_counter_;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

    rclcpp::Subscription<px4_msgs::msg::VehicleAttitude>::SharedPtr vehicle_attitude_subscription_;
    void vehicle_attitude_subscription_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg);
    Eigen::Quaternionf q = Eigen::Quaternionf::Identity();
    Eigen::Vector3f v_body{0.0f, 1.0f, 0.0f};
    Eigen::Vector3f a_body{1.0f, 0.0f, 0.0f};
    Eigen::Vector3f v_ned;
    Eigen::Vector3f a_ned;
    std::atomic_bool have_attitude_{false};
};

void CircleTest::vehicle_attitude_subscription_callback(const px4_msgs::msg::VehicleAttitude::UniquePtr msg) {
    q = Eigen::Quaternionf(msg->q[0], msg->q[1], msg->q[2], msg->q[3]);
    have_attitude_ = true;
    //RCLCPP_INFO(this->get_logger(), "q = (%f, %f, %f, %f)\n", q.w(), q.x(), q.y(), q.z());
}

void CircleTest::publish_vehicle_command(uint16_t command, float param1, float param2) {
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

void CircleTest::publish_offboard_control_mode() {
    px4_msgs::msg::OffboardControlMode msg{};
    msg.position = false;
	msg.velocity = true;
	msg.acceleration = true;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

// vy = 1.0; ax = vy^2 / R = 1.0; R = 1.0;
// void CircleTest::publish_vehicle_local_position_setpoint() {
//     px4_msgs::msg::VehicleLocalPositionSetpoint msg{};
//     msg.vy = 1.0;
//     msg.acceleration = {1.0, 0.0, 0.0};
//     msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
//     vehicle_local_position_setpoint_publisher_->publish(msg);
// }
void CircleTest::publish_trajectory_setpoint() {
    if (!have_attitude_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "No attitude yet, skipping trajectory setpoint");
        return;
    }
    v_ned = q * v_body;
    a_ned = q * a_body;
    px4_msgs::msg::TrajectorySetpoint msg{};
    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    msg.velocity = {v_ned.x(), v_ned.y(), v_ned.z()};
    msg.acceleration = {a_ned.x(), a_ned.y(), a_ned.z()};
    trajectory_setpoint_publisher_->publish(msg);
}

void CircleTest::arm() {
    publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
	RCLCPP_INFO(this->get_logger(), "Arm command send");
}

int main(int argc, char *argv[]) {
    std::cout << "Starting offboard control mode..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CircleTest>());

    rclcpp::shutdown();
    return 0;
}