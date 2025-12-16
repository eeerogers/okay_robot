#include "okay_robot_servo/servo_bus_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ServoBusNode>());
    rclcpp::shutdown();

    return 0;
}