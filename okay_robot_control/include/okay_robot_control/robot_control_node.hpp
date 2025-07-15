#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_control/tidybot_state.hpp"
#include "okay_robot_msgs/msg/tidy_bot_cmd.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode();

private:
    void timer_callback();
    void tidybot_command_subscriber_callback(const geometry_msgs::msg::Twist::SharedPtr msg);
    void tidybot_state_subscriber_callback(const okay_robot_msgs::msg::TidyBotCmd::SharedPtr msg);

    double control_freq_;
    TidyBotState current_state_;
    TidyBotGoal current_goal_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::TidyBotCmd>::SharedPtr publisher_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr command_subscriber_;
    rclcpp::Subscription<okay_robot_msgs::msg::TidyBotCmd>::SharedPtr state_subscriber_;
};