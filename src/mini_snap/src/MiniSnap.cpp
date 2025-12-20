#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include "mini_snap/ToFly.hpp"

int main(int argc, char *argv[]) {
    //std::cout<<"Hello world!"<<std::endl;
    std::cout << "Starting offboard control mode..." << std::endl;
    setvbuf(stdout, NULL, _IONBF, BUFSIZ);
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ToFly>());

    rclcpp::shutdown();

    return 0;
}