#include "okay_robot_view/view_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OkayRobot::ViewNode>());
    rclcpp::shutdown();

    return 0;
}