#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_common/print_data.hpp"
#include "okay_robot_control/control/direct_controller.hpp"
#include "okay_robot_control/robot_control_node.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

RobotControlNode::RobotControlNode()
    : Node("robot_control_node")
{
    // TODO: move this out to a config
    this->control_freq_ = 30.0;
    this->controller_ = std::make_unique<DirectController>();
    this->kinematics_ = std::make_unique<Kinematics>();

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->control_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&RobotControlNode::timer_callback_, this));

    // set up pubs/subs
    this->servo_bus_command_publisher_
        = this->create_publisher<okay_robot_msgs::msg::ServoBusCommand>("servo_bus_command", 10);
    this->okay_robot_goal_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::OkayRobotGoal>("okay_robot_goal", 10,
            std::bind(&RobotControlNode::okay_robot_goal_subscriber_callback_, this, _1));
    this->servo_bus_observation_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
            "servo_bus_observation", 10,
            std::bind(&RobotControlNode::servo_bus_observation_subscriber_callback_, this, _1));

    // initialize current state
    auto current_time = std::chrono::steady_clock::now();
    this->last_observation_ = std::make_unique<OkayRobot::Observation>(
        current_time, std::vector<float>(7, 0.0), std::vector<float>(7, 0.0));
}

void RobotControlNode::timer_callback_()
{
    // spin control loop
    OkayRobot::Command next_command
        = this->controller_->step_control_loop(*this->last_observation_.get());

    // send command to servo bus
    okay_robot_msgs::msg::ServoBusCommand bus_command
        = this->okay_robot_to_servo_bus_command_(next_command);
    this->servo_bus_command_publisher_->publish(bus_command);
}

okay_robot_msgs::msg::ServoBusCommand RobotControlNode::okay_robot_to_servo_bus_command_(
    OkayRobot::Command& command)
{
    auto bus_command = okay_robot_msgs::msg::ServoBusCommand();
    for (int i = 0; i < command.joint_positions.size(); i++) {
        auto servo_command = okay_robot_msgs::msg::ServoCommand();
        servo_command.id = i + 1;
        servo_command.position = command.joint_positions[i];
        servo_command.enable = true;

        bus_command.commands.push_back(servo_command);
    }

    return bus_command;
}

void RobotControlNode::okay_robot_goal_subscriber_callback_(
    const okay_robot_msgs::msg::OkayRobotGoal msg)
{
    // catch robot command
    OkayRobot::Pose new_goal(msg.joint_positions);
    this->controller_->set_goal_state(new_goal);
}

void RobotControlNode::servo_bus_observation_subscriber_callback_(
    const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg)
{
    auto current_time = std::chrono::steady_clock::now();
    std::vector<float> joint_positions(this->last_observation_->joint_positions);

    for (auto observation : msg->observations) {
        if (observation.id > 7) {
            RCLCPP_WARN(this->get_logger(), "joint%d out of range: only %d total joints",
                observation.id, 7);
            continue;
        }

        joint_positions[observation.id - 1] = observation.position;
    }

    OkayRobot::Observation new_observation(
        current_time, joint_positions, std::vector<float>(7, 0.0));

    OkayRobot::Transform new_transform
        = this->kinematics_->get_forward(OkayRobot::Pose(new_observation.joint_positions));
    std::string joints_string = vec_to_string(new_observation.joint_positions);

    RCLCPP_INFO(this->get_logger(),
        "current position:\npose: [%s]\n\tx: %f\n\ty: %f\n\tz: %f\n----------",
        joints_string.c_str(), new_transform.x, new_transform.y, new_transform.z);

    this->last_observation_ = std::make_unique<OkayRobot::Observation>(new_observation);
}