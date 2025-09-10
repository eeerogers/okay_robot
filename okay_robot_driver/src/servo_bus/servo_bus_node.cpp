#include "okay_robot_driver/servo_bus/servo_bus_node.hpp"
#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"

using std::placeholders::_1;

ServoBusNode::ServoBusNode()
    : Node("servo_bus_node")
{
    this->servo_bus_.init(this->port_, this->baud_);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->poll_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&ServoBusNode::timer_callback_, this));

    this->publisher_ = this->create_publisher<okay_robot_msgs::msg::ServoBusObservation>(
        "servo_observation", 10);
    this->subscriber_ = this->create_subscription<okay_robot_msgs::msg::ServoBusCommand>(
        "servo_command", 10, std::bind(&ServoBusNode::command_callback_, this, _1));
}

// TODO: implement
void ServoBusNode::timer_callback_()
{
    // get observation from servo bus and publish to ros
}

// TODO: implement
void ServoBusNode::command_callback_(const okay_robot_msgs::msg::ServoBusCommand msg)
{
    // turn commands into packets and send to servo bus
}