#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_control/robot_control_node.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotControlNode::RobotControlNode()
    : Node("robot_control_node")
{
    // TODO: move this out to a config
    this->control_freq_ = 30.0;

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->control_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&RobotControlNode::timer_callback, this));

    // set up pubs/subs
    this->servo_bus_command_publisher_
        = this->create_publisher<okay_robot_msgs::msg::ServoBusCommand>("okay_robot_command", 10);
    this->command_subscriber_
        = this->create_subscription<geometry_msgs::msg::Twist>("okay_robot_goal", 10,
            std::bind(&RobotControlNode::servo_bus_command_subscriber_callback, this, _1));
    this->servo_bus_observation_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
            "okay_robot_observation", 10,
            std::bind(&RobotControlNode::servo_bus_observation_subscriber_callback, this, _1));

    // initialize current state
    auto current_time = std::chrono::steady_clock::now();
    this->last_observation_ = std::make_unique<OkayRobotObservation>(
        current_time, Eigen::VectorXd::Zero(7), Eigen::VectorXd::Zero(7));
}

void RobotControlNode::timer_callback()
{
    // spin control loop

    // print the state for now
    std::string state = std::accumulate(this->last_observation_->joint_positions.begin(),
        this->last_observation_->joint_positions.end(), std::string(),
        [](std::string& s, float n) { return s + (s.empty() ? "" : " ") + std::to_string(n); });

    RCLCPP_WARN(this->get_logger(), "[ %s ]", state.c_str());
}

void RobotControlNode::servo_bus_command_subscriber_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
{
    // catch robot command
}

void RobotControlNode::servo_bus_observation_subscriber_callback(
    const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg)
{
    auto current_time = std::chrono::steady_clock::now();
    Eigen::VectorXd joint_positions(this->last_observation_->joint_positions);

    for (auto observation : msg->observations) {
        if (observation.id > 7) {
            RCLCPP_WARN(this->get_logger(), "joint%d out of range: only %d total joints",
                observation.id, 7);
            continue;
        }

        joint_positions[observation.id - 1] = observation.position;
    }

    OkayRobotObservation new_observation(current_time, joint_positions, Eigen::VectorXd::Zero(7));
    this->last_observation_ = std::make_unique<OkayRobotObservation>(new_observation);
}