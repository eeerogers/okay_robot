#include "okay_robot_control/robot_control_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OkayRobot::RobotControlNode>());
    rclcpp::shutdown();

    return 0;
}