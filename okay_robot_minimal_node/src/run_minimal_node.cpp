#include "okay_robot_minimal_node/minimal_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MinimalNode>("minimal_node"));
    rclcpp::shutdown();

    return 0;
}