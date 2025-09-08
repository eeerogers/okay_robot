#pragma once

#include "okay_robot_driver/servo_bus.hpp"
#include "rclcpp/rclcpp.hpp"

class ServoBusNode : public rclcpp::Node {
public:
    ServoBusNode();

private:
    void timer_callback_();

    double poll_freq_ = 100.0;
    std::string port_ = "dev/ttyACM0";
    LibSerial::BaudRate baud_ = LibSerial::BaudRate::BAUD_1000000;

    ServoBus servo_bus_;
    rclcpp::TimerBase::SharedPtr timer_;
};