#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_control/robot_control_node.hpp"
#include "okay_robot_control/tidybot_state.hpp"
#include "okay_robot_msgs/msg/tidy_bot_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotControlNode::RobotControlNode()
    : Node("robot_control_node")
{
    // TODO: move this out to a config
    this->control_freq_ = 30.0;

    // init current robot state to zeros
    this->current_state_ = { { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 }, 0.0, 0.0, 0.0, 0.0 };

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->control_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&RobotControlNode::timer_callback, this));

    // set up pubs/subs
    this->publisher_ = this->create_publisher<okay_robot_msgs::msg::TidyBotCmd>("tidybot_cmd", 10);
    this->command_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>("tidybot_goal",
        10, std::bind(&RobotControlNode::tidybot_command_subscriber_callback, this, _1));
    this->state_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::TidyBotCmd>("tidybot_state", 10,
            std::bind(&RobotControlNode::tidybot_state_subscriber_callback, this, _1));
}

// TODO: do actual control here instead of hacky mapping
void RobotControlNode::timer_callback()
{
    // turn goal state into joint commands
    okay_robot_msgs::msg::TidyBotCmd new_cmd;
    std::array<float, 7> joint_cmd = { this->current_goal_.r, this->current_goal_.p,
        this->current_goal_.yaw, 0.0, 0.0, 0.0, 0.0 };

    // generate control command
    new_cmd.joints = joint_cmd;
    new_cmd.gripper = 0.0;
    new_cmd.x_pos = this->current_goal_.x;
    new_cmd.y_pos = this->current_goal_.y;
    new_cmd.theta = this->current_goal_.z;

    // publish control command
    this->publisher_->publish(new_cmd);
}

void RobotControlNode::tidybot_command_subscriber_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // save robot command data
    this->current_goal_.x = msg->linear.x;
    this->current_goal_.y = msg->linear.y;
    this->current_goal_.z = msg->linear.z;
    this->current_goal_.r = msg->angular.x;
    this->current_goal_.p = msg->angular.y;
    this->current_goal_.yaw = msg->angular.z;
}

void RobotControlNode::tidybot_state_subscriber_callback(
    const okay_robot_msgs::msg::TidyBotCmd::SharedPtr msg)
{
    // save robot state data
    this->current_state_.joints = msg->joints;
    this->current_state_.gripper = msg->gripper;
    this->current_state_.x_pos = msg->x_pos;
    this->current_state_.y_pos = msg->y_pos;
    this->current_state_.theta = msg->theta;
}