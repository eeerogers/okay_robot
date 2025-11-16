#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_common/print_data.hpp"
#include "okay_robot_common/topic.hpp"
#include "okay_robot_common/transform.hpp"
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
    this->controller_ = std::make_unique<DirectController>();
    this->kinematics_ = std::make_unique<Kinematics>();

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->control_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&RobotControlNode::timer_callback_, this));

    // set up pubs/subs
    this->servo_bus_command_publisher_
        = this->create_publisher<okay_robot_msgs::msg::ServoBusCommand>(
            TOPIC_SERVO_BUS_COMMAND, 10);
    this->okay_robot_goal_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::OkayRobotGoal>(TOPIC_OKAY_ROBOT_GOAL, 10,
            std::bind(&RobotControlNode::okay_robot_goal_subscriber_callback_, this, _1));
    this->twist_subscriber_
        = this->create_subscription<geometry_msgs::msg::Twist>(TOPIC_OKAY_ROBOT_GOAL_TWIST, 10,
            std::bind(&RobotControlNode::twist_subscriber_callback_, this, _1));
    this->servo_bus_observation_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
            TOPIC_SERVO_BUS_OBSERVATION, 10,
            std::bind(&RobotControlNode::servo_bus_observation_subscriber_callback_, this, _1));
    this->gamepad_command_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::GamepadCommand>(TOPIC_GAMEPAD, 10,
            std::bind(&RobotControlNode::gamepad_command_subscriber_callback_, this, _1));

    // initialize current state
    auto current_time = std::chrono::steady_clock::now();
    this->last_observation_ = std::make_unique<OkayRobot::Observation>(current_time,
        std::vector<float>({ 1.57, 1.57, 4.71, 3.14, 1.57, 3.14, 0.15 }),
        std::vector<float>(7, 0.0));

    OkayRobot::Transform step_tf
        = this->kinematics_->get_forward(OkayRobot::Pose(this->last_observation_->joint_positions));
    this->last_step_ = std::make_unique<OkayRobot::Transform>(step_tf);
    this->next_step_ = std::make_unique<OkayRobot::Transform>(step_tf);
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

    // update state variables
    this->last_step_ = std::make_unique<OkayRobot::Transform>(*this->next_step_.get());
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
    this->set_goal_pose_(new_goal);
}

void RobotControlNode::twist_subscriber_callback_(const geometry_msgs::msg::Twist msg)
{
    // convert to transform
    Eigen::Vector3f position(msg.linear.x, msg.linear.y, msg.linear.z);
    Eigen::Matrix3f rotation
        = OkayRobot::euler_to_rotation(msg.angular.x, msg.angular.y, msg.angular.z);
    OkayRobot::Transform twist_tf(position, rotation);

    // calculate inverse kinematics
    OkayRobot::Pose robot_pose(this->kinematics_->get_inverse(
        twist_tf, OkayRobot::Pose(this->last_observation_->joint_positions)));

    // send joint positions to robot
    this->set_goal_pose_(robot_pose);
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
    this->last_observation_ = std::make_unique<OkayRobot::Observation>(new_observation);
}

void RobotControlNode::gamepad_command_subscriber_callback_(
    const okay_robot_msgs::msg::GamepadCommand::SharedPtr msg)
{
    Eigen::Vector3f xyz = Eigen::Vector3f::Zero();
    Eigen::Vector3f rpy = Eigen::Vector3f::Zero();

    // stick range: (-32768, 32767)

    // linear
    // x: left stick y-axis
    // y: left stick x-axis
    // z: L2/R2

    // shift left/right trigger range to positive
    int left_trigger(msg->l_trigger + 32768);
    int right_trigger(msg->r_trigger + 32768);

    xyz(0) = -msg->left_stick_y_axis;
    xyz(1) = -msg->left_stick_x_axis;
    if (left_trigger >= right_trigger)
        xyz(2) = -left_trigger / 2.0;
    else
        xyz(2) = right_trigger / 2.0;

    if (std::abs(xyz(0)) <= this->gamepad_stick_deadzone_)
        xyz(0) = 0.0;
    if (std::abs(xyz(1)) <= this->gamepad_stick_deadzone_)
        xyz(1) = 0.0;
    if (std::abs(xyz(2)) <= this->gamepad_trigger_deadzone_)
        xyz(2) = 0.0;

    // angular
    // roll: x/b button
    // pitch: right stick y-axis
    // yaw: right stick x-axis

    rpy(0) = -msg->right_stick_x_axis;
    rpy(1) = msg->right_stick_y_axis;
    if (msg->b_button) {
        rpy(2) = 32767.0;
    } else if (msg->x_button) {
        rpy(2) = -32768.0;
    }

    if (std::abs(rpy(0)) <= this->gamepad_stick_deadzone_)
        rpy(0) = 0.0;
    if (std::abs(rpy(1)) <= this->gamepad_stick_deadzone_)
        rpy(1) = 0.0;

    xyz *= (this->gamepad_speed_linear_ / 32768.0);
    rpy *= (this->gamepad_speed_angular_ / 32768.0);

    if (xyz.norm() == 0.0 && rpy.norm() == 0.0) {
        return;
    }

    // increment step in gamepad direction
    Eigen::Vector3f position = this->last_step_->position() + xyz;
    Eigen::Matrix3f rotation
        = this->last_step_->rotation() * OkayRobot::euler_to_rotation(rpy[0], rpy[1], rpy[2]);

    this->next_step_ = std::make_unique<OkayRobot::Transform>(position, rotation);
    auto goal_pose = this->kinematics_->get_inverse(
        *this->next_step_.get(), OkayRobot::Pose(this->last_observation_->joint_positions));

    // update controller goal state
    this->controller_->set_goal_state(goal_pose);
}

void RobotControlNode::set_goal_pose_(const OkayRobot::Pose& pose)
{
    bool is_valid = OkayRobot::pose_is_valid(pose);
    std::string joints_string = vec_to_string(pose.joint_positions);
    RCLCPP_INFO(this->get_logger(), "current pose is %s:\n[%s]", is_valid ? "valid" : "invalid",
        joints_string.c_str());

    if (!is_valid)
        return;

    this->controller_->set_goal_state(pose);
}