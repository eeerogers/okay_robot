#pragma once

#include "mujoco/mujoco.h"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <mutex>

class MujocoSimNode : public rclcpp::Node {
public:
    MujocoSimNode();
    ~MujocoSimNode();

private:
    void timer_callback();
    void servo_bus_subscriber_callback(const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg);

    mjModel* m_;
    mjData* d_;
    std::mutex mutex_;
    std::atomic<bool> gui_shutdown_flag_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr publisher_;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr
        servo_bus_command_subscriber_;

    std::thread gui_thread_;
};
