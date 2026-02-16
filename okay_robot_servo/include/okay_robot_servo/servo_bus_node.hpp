#pragma once

#include <queue>

#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "okay_robot_servo/servo_bus.hpp"
#include "rclcpp/rclcpp.hpp"

namespace OkayRobot {
class ServoBusNode : public rclcpp::Node {
public:
    ServoBusNode();

private:
    void timerCallback_();
    void servoBusCommandCallback_(const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg);

    void publishObservation_();
    void executeNextCommand_();

    std::queue<okay_robot_msgs::msg::ServoBusCommand> command_queue_;

    const double poll_freq_ = 100.0;
    const std::string port_ = "/dev/ttyACM0";
    const LibSerial::BaudRate baud_ = LibSerial::BaudRate::BAUD_1000000;

    const float rad_to_range_ = 4095.0 / (2.0 * M_PI);

    ServoBus servo_bus_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr publisher_;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr subscriber_;
};
}