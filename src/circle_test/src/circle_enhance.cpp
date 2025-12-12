#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/offboard_control_mode.hpp"
#include "px4_msgs/msg/trajectory_setpoint.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_global_position.hpp"

#include <chrono>
#include <iostream>
#include <atomic>

class CircleTest : public rclcpp::Node {
public:
    CircleTest() : Node("circle_test") {
        offboard_control_mode_publisher_ = this->create_publisher<px4_msgs::msg::OffboardControlMode>("/fmu/in/offboard_control_mode", 10);
        trajectory_setpoint_publisher_ = this->create_publisher<px4_msgs::msg::TrajectorySetpoint>("/fmu/in/trajectory_setpoint", 10);
        vehicle_command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>("/fmu/in/vehicle_command", 10);

        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_global_position_subscription_ = this->create_subscription<px4_msgs::msg::VehicleGlobalPosition>(
            "/fmu/out/vehicle_global_position", qos,
            std::bind(&CircleTest::vehicle_global_position_subscription_callback, this, std::placeholders::_1)
        );

        offboard_setpoint_counter_ = 0;
        auto timer_callback = [this]() -> void {
            if (offboard_setpoint_counter_ == 10) {
                this->publish_vehicle_command(px4_msgs::msg::VehicleCommand::VEHICLE_CMD_DO_SET_MODE, 1, 6);
                this->arm();
            }

            if(is_enough) {
                publish_offboard_control_mode();
                publish_trajectory_setpoint();
            }
            else {
                publish_offboard_control_mode();
                height_call();
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

    std::atomic<uint64_t> timestamp_;

    uint64_t offboard_setpoint_counter_;

    void publish_vehicle_command(uint16_t command, float param1 = 0.0, float param2 = 0.0);
    void publish_offboard_control_mode();
    void publish_trajectory_setpoint();

    float t = 0.0f;
    float v = 3.0f;
    float r = 3.0f;
    float omega = v / r;

    rclcpp::Subscription<px4_msgs::msg::VehicleGlobalPosition>::SharedPtr vehicle_global_position_subscription_;
    void vehicle_global_position_subscription_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg);
    std::atomic_bool is_enough{false};
    void height_call() {
        px4_msgs::msg::TrajectorySetpoint msg{};
        msg.position = {0.0, 0.0, -5.0};
        //msg.yaw = -3.14; // [-PI:PI]
        msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
        trajectory_setpoint_publisher_->publish(msg);
    }
};

void CircleTest::vehicle_global_position_subscription_callback(const px4_msgs::msg::VehicleGlobalPosition::UniquePtr msg) {
    if(msg->alt > 4.9) is_enough = true;
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
    msg.position = true;
	msg.velocity = false;
	msg.acceleration = false;
	msg.attitude = false;
	msg.body_rate = false;
	msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
	offboard_control_mode_publisher_->publish(msg);
}

void CircleTest::publish_trajectory_setpoint() {
    px4_msgs::msg::TrajectorySetpoint msg{};

    float px = r * cos(omega * t);
    float py = r * sin(omega * t);
    msg.position = {px, py, -5.0};
    msg.yaw = atan2(py, px);

    msg.timestamp = this->get_clock()->now().nanoseconds() / 1000;
    trajectory_setpoint_publisher_->publish(msg);
    t+=0.1;
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