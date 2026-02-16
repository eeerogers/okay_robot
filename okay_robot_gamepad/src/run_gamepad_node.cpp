#include "okay_robot_gamepad/gamepad_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OkayRobot::GamepadNode>());
    rclcpp::shutdown();

    return 0;
}