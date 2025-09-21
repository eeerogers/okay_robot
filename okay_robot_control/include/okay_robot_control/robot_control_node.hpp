#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_control/control_algorithms/control_loop.hpp"
#include "okay_robot_control/okay_robot_state.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode();

private:
    void timer_callback();
    void servo_bus_command_subscriber_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void servo_bus_observation_subscriber_callback(
        const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg);

    double control_freq_;
    std::unique_ptr<ControlLoop> control_loop_;
    std::unique_ptr<OkayRobotState> current_state_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr
        servo_bus_command_publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr
        servo_bus_observation_subscriber_;
};