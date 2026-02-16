#include "okay_robot_sim/mujoco_sim_node.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OkayRobot::MujocoSimNode>());
    rclcpp::shutdown();

    return 0;
}