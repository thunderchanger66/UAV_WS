#include <rclcpp/rclcpp.hpp>

#include "px4_msgs/msg/vehicle_odometry.hpp"
#include <nav_msgs/msg/path.hpp>

class PosRviz : public rclcpp::Node {
public:
    PosRviz() : Node("posrviz") {
        rmw_qos_profile_t qos_profile = rmw_qos_profile_sensor_data;
        auto qos = rclcpp::QoS(rclcpp::QoSInitialization(qos_profile.history, 5), qos_profile);
        vehicle_odometry_sub_ = this->create_subscription<px4_msgs::msg::VehicleOdometry>(
            "fmu/out/vehicle_odometry", qos, 
            std::bind(&PosRviz::vehicle_odometry_sub_callback, this, std::placeholders::_1)
        );

        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("px4_path", 10);
        path_.header.frame_id = "map";
        RCLCPP_INFO(this->get_logger(), "Path Rviz OK!");
    }

private:
    rclcpp::Subscription<px4_msgs::msg::VehicleOdometry>::SharedPtr vehicle_odometry_sub_;
    void vehicle_odometry_sub_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg);

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_;
};

void PosRviz::vehicle_odometry_sub_callback(const px4_msgs::msg::VehicleOdometry::UniquePtr msg) {
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "map";
    pose.header.stamp = this->now();

    pose.pose.position.x = msg->position[0];
    pose.pose.position.y = -msg->position[1];
    pose.pose.position.z = -msg->position[2];

    path_.header.stamp = pose.header.stamp;
    path_.poses.push_back(pose);
    path_pub_->publish(path_);
}

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PosRviz>());
    rclcpp::shutdown();
    return 0;
}