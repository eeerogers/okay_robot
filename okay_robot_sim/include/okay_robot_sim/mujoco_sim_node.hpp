#pragma once

#include "mujoco/mujoco.h"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class MujocoSimNode : public rclcpp::Node {
public:
    MujocoSimNode();
    ~MujocoSimNode();

private:
    void timer_callback();

    mjModel* m_;
    mjData* d_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    std::thread gui_thread_;
};
