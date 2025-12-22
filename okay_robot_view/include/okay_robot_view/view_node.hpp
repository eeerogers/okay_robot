#pragma once

#include <GLFW/glfw3.h>
#include <atomic>
#include <mutex>

#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "okay_robot_view/circular_buffer.hpp"
#include "rclcpp/rclcpp.hpp"

class ViewNode : public rclcpp::Node {
public:
    ViewNode();
    ~ViewNode();

private:
    void timer_callback_();
    void servo_bus_observation_subscriber_callback_(
        const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg);

    std::mutex mutex_;
    std::atomic<bool> gui_shutdown_flag_;
    std::thread gui_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    const double poll_freq_ = 60.0;

    const int buffer_size_ = 1000;
    CircularBuffer<okay_robot_msgs::msg::ServoBusObservation::SharedPtr> observations_
        = CircularBuffer<okay_robot_msgs::msg::ServoBusObservation::SharedPtr>(this->buffer_size_);

    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr
        servo_bus_observation_subscriber_
        = nullptr;
};
