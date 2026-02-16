#pragma once

#include "mujoco/mujoco.h"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"
#include <atomic>
#include <mutex>

namespace OkayRobot {
class MujocoSimNode : public rclcpp::Node {
public:
    MujocoSimNode();
    ~MujocoSimNode();

private:
    void timerCallback_();
    void publishObservations_();
    void servoBusCommandSubscriberCallback_(
        const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg);
    void servoBusObservationSubscriberCallback_(
        const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg);

    mjModel* m_;
    mjData* d_;
    std::mutex mutex_;
    std::atomic<bool> gui_shutdown_flag_;
    std::thread gui_thread_;

    rclcpp::TimerBase::SharedPtr timer_;

    // pub/sub for sim mode
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr
        servo_bus_observation_publisher_
        = nullptr;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr
        servo_bus_command_subscriber_
        = nullptr;

    // pub/sub for puppet mode
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr
        servo_bus_observation_subscriber_
        = nullptr;
};
}