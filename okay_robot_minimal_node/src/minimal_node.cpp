#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_minimal_node/minimal_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

MinimalNode::MinimalNode(const char* name)
    : Node(name)
    , count_(0)
{
    this->publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
    this->timer_ = this->create_wall_timer(500ms, std::bind(&MinimalNode::timer_callback, this));
}

void MinimalNode::timer_callback()
{
    auto message = std_msgs::msg::String();
    message.data = "500ms count: " + std::to_string(this->count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    this->publisher_->publish(message);
}