#pragma once

#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"

class ServoBusNode : public rclcpp::Node {
public:
    ServoBusNode();

private:
    void timer_callback_();
    void command_callback_(const okay_robot_msgs::msg::ServoBusCommand msg);

    double poll_freq_ = 100.0;
    std::string port_ = "dev/ttyACM0";
    LibSerial::BaudRate baud_ = LibSerial::BaudRate::BAUD_1000000;

    ServoBus servo_bus_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr publisher_;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr subscriber_;
};