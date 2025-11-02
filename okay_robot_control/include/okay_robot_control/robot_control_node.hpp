#pragma once

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_common/okay_robot_state.hpp"
#include "okay_robot_control/control/controller.hpp"
#include "okay_robot_control/kinematics.hpp"
#include "okay_robot_msgs/msg/okay_robot_goal.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "rclcpp/rclcpp.hpp"

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode();

private:
    // callbacks
    void timer_callback_();
    void okay_robot_goal_subscriber_callback_(const okay_robot_msgs::msg::OkayRobotGoal msg);
    void twist_subscriber_callback_(const geometry_msgs::msg::Twist msg);
    void servo_bus_observation_subscriber_callback_(
        const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg);

    okay_robot_msgs::msg::ServoBusCommand okay_robot_to_servo_bus_command_(
        OkayRobot::Command& command);

    double control_freq_;
    std::unique_ptr<Controller> controller_;
    std::unique_ptr<Kinematics> kinematics_;
    std::unique_ptr<OkayRobot::Observation> last_observation_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::ServoBusCommand>::SharedPtr
        servo_bus_command_publisher_;
    rclcpp::Subscription<okay_robot_msgs::msg::OkayRobotGoal>::SharedPtr
        okay_robot_goal_subscriber_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_subscriber_;
    rclcpp::Subscription<okay_robot_msgs::msg::ServoBusObservation>::SharedPtr
        servo_bus_observation_subscriber_;
};