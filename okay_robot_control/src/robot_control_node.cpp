#include <Eigen/Dense>
#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "okay_robot_common/print_data.hpp"
#include "okay_robot_common/topic.hpp"
#include "okay_robot_common/transform/transform.hpp"
#include "okay_robot_control/control/direct_controller.hpp"
#include "okay_robot_control/robot_control_node.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace OkayRobot {

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
        = this->create_wall_timer(duration, std::bind(&RobotControlNode::timerCallback_, this));

    // set up pubs/subs
    this->servo_bus_command_publisher_
        = this->create_publisher<okay_robot_msgs::msg::ServoBusCommand>(
            OkayRobotTopic::SERVO_BUS_COMMAND, 10);
    this->okay_robot_goal_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::OkayRobotGoal>(
            OkayRobotTopic::OKAY_ROBOT_GOAL, 10,
            std::bind(&RobotControlNode::goalSubscriberCallback_, this, _1));
    this->twist_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        OkayRobotTopic::OKAY_ROBOT_GOAL_TWIST, 10,
        std::bind(&RobotControlNode::twistSubscriberCallback_, this, _1));
    this->servo_bus_observation_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
            OkayRobotTopic::SERVO_BUS_OBSERVATION, 10,
            std::bind(&RobotControlNode::servoBusObservationSubscriberCallback_, this, _1));
    this->gamepad_command_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::GamepadCommand>(OkayRobotTopic::GAMEPAD,
            10, std::bind(&RobotControlNode::gamepadCommandSubscriberCallback_, this, _1));

    // initialize current state
    auto current_time = std::chrono::steady_clock::now();
    this->last_observation_ = std::make_unique<Observation>(Observation { current_time,
        std::vector<float>({ M_PI_2f, M_PI_2f, 3.0 * M_PI_2f, M_PIf, M_PI_2f, M_PIf, M_PI_4f }),
        std::vector<float>(7, 0.0) });

    Transform step_tf
        = this->kinematics_->getForward(JointPose(this->last_observation_->joint_positions));
    auto new_state = GamepadState(step_tf, M_PI_4f);
    this->last_step_ = std::make_unique<GamepadState>(new_state);
    this->next_step_ = std::make_unique<GamepadState>(new_state);
}

void RobotControlNode::timerCallback_()
{
    // spin control loop
    JointPose next_command = this->controller_->stepControlLoop(*this->last_observation_.get());

    // send command to servo bus
    okay_robot_msgs::msg::ServoBusCommand bus_command
        = this->jointPoseToServoBusCommand_(next_command);
    this->servo_bus_command_publisher_->publish(bus_command);

    // update state variables
    this->last_step_ = std::make_unique<GamepadState>(*this->next_step_.get());
}

okay_robot_msgs::msg::ServoBusCommand RobotControlNode::jointPoseToServoBusCommand_(
    JointPose& command)
{
    auto bus_command = okay_robot_msgs::msg::ServoBusCommand();
    for (int i = 0; i < command.joint_positions.size(); i++) {
        okay_robot_msgs::msg::ServoCommand servo_command;
        servo_command.id = i + 1;
        servo_command.position = command.joint_positions[i];
        servo_command.enable = true;

        bus_command.commands.push_back(servo_command);
    }

    // just hack this in here for now
    okay_robot_msgs::msg::ServoCommand eef_command;
    eef_command.id = 7;
    eef_command.position = this->next_step_->eef_position;
    eef_command.enable = true;
    bus_command.commands.push_back(eef_command);

    return bus_command;
}

void RobotControlNode::goalSubscriberCallback_(const okay_robot_msgs::msg::OkayRobotGoal msg)
{
    // catch robot command
    JointPose new_goal(msg.joint_positions);
    this->setGoalPose_(new_goal);
}

void RobotControlNode::twistSubscriberCallback_(const geometry_msgs::msg::Twist msg)
{
    // convert to transform
    Position position(msg.linear.x, msg.linear.y, msg.linear.z);
    Rotation rotation(msg.angular.x, msg.angular.y, msg.angular.z);
    Transform twist_tf(position, rotation);

    // calculate inverse kinematics
    JointPose robot_pose(this->kinematics_->getInverse(
        twist_tf, JointPose(this->last_observation_->joint_positions)));

    // send joint positions to robot
    this->setGoalPose_(robot_pose);
}

void RobotControlNode::servoBusObservationSubscriberCallback_(
    const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg)
{
    auto current_time = std::chrono::steady_clock::now();
    std::vector<float> joint_pos(this->last_observation_->joint_positions);
    std::vector<float> joint_vel(this->last_observation_->joint_velocities);
    std::vector<float> joint_lod(this->last_observation_->joint_loads);

    // TODO: un-hardcode 7 here
    for (auto observation : msg->observations) {
        if (observation.id > 7) {
            RCLCPP_WARN(this->get_logger(), "joint%d out of range: only %d total joints",
                observation.id, 7);
            continue;
        }

        joint_pos[observation.id - 1] = observation.position;
        joint_vel[observation.id - 1] = observation.speed;
        joint_lod[observation.id - 1] = observation.load;
    }

    Observation new_observation { current_time, joint_pos, joint_vel, joint_lod };
    this->last_observation_ = std::make_unique<Observation>(new_observation);

    // test jacobian
    Eigen::Matrix<float, 6, 6> jacobian
        = this->kinematics_->getJacobian(JointPose(new_observation.joint_positions));
    Eigen::Vector<float, 6> joint_velocities(std::vector<float>(
        new_observation.joint_velocities.begin(), new_observation.joint_velocities.begin() + 6)
            .data());
    Eigen::Vector<float, 6> joint_positions(joint_pos.data());
    Eigen::Vector<float, 6> eef_velocity = jacobian * joint_velocities;

    RCLCPP_INFO_STREAM(this->get_logger(), eef_velocity);
}

void RobotControlNode::gamepadCommandSubscriberCallback_(
    const okay_robot_msgs::msg::GamepadCommand::SharedPtr msg)
{
    Eigen::Vector3f xyz = Eigen::Vector3f::Zero();
    Eigen::Vector3f rpy = Eigen::Vector3f::Zero();
    float eef_dir = 0.0;

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

    // eef
    // open/close: l1/r1

    if (msg->l1_button) {
        eef_dir = 1.0;
    } else if (msg->r1_button) {
        eef_dir = -1.0;
    }

    if (xyz.norm() == 0.0 && rpy.norm() == 0.0 && eef_dir == 0.0)
        return;

    // increment step in gamepad direction
    Position position(this->last_step_->position().vector() + xyz);
    Rotation rotation(
        this->last_step_->rotation().matrix() * Rotation(rpy[0], rpy[1], rpy[2]).matrix());
    float gripper_position
        = this->last_step_->eef_position + (eef_dir * this->gamepad_speed_angular_);
    gripper_position = std::clamp(gripper_position, (float)0.0, M_PI_2f);

    auto next_step_tf = Transform(position, rotation);
    JointPose goal_pose = this->kinematics_->getInverse(
        next_step_tf, JointPose(this->last_observation_->joint_positions));

    // update next step
    this->next_step_ = std::make_unique<GamepadState>(next_step_tf, gripper_position);

    // update controller goal state
    this->setGoalPose_(goal_pose);
}

void RobotControlNode::setGoalPose_(const JointPose& pose)
{
    if (this->kinematics_->poseIsValid(pose)) {
        this->controller_->setGoalState(pose);
    }
}

}