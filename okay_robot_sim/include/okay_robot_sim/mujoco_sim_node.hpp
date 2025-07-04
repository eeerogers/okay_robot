#pragma once

#include "mujoco/mujoco.h"
#include "okay_robot_msgs/msg/tidy_bot_cmd.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include <mutex>

class MujocoSimNode : public rclcpp::Node {
public:
    MujocoSimNode();
    ~MujocoSimNode();

private:
    void timer_callback();
    void tidy_bot_subscriber_callback(const okay_robot_msgs::msg::TidyBotCmd::SharedPtr msg);

    mjModel* m_;
    mjData* d_;
    std::mutex mutex_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::Subscription<okay_robot_msgs::msg::TidyBotCmd>::SharedPtr tidy_bot_subscriber_;

    std::thread gui_thread_;
};
