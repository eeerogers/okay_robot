#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_common/topic.hpp"
#include "okay_robot_description/descriptions.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "okay_robot_msgs/msg/servo_observation.hpp"
#include "okay_robot_sim/mujoco_gui.hpp"
#include "okay_robot_sim/mujoco_sim_node.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

MujocoSimNode::MujocoSimNode()
    : Node("mujoco_sim_node")
{
    // declare ros parameters
    this->declare_parameter<bool>("run_as_puppet", false);
    bool run_as_puppet = this->get_parameter("run_as_puppet").as_bool();

    RCLCPP_INFO(this->get_logger(), "Run as puppet: %s", run_as_puppet ? "true" : "false");

    // initialize mujoco environment
    this->m_ = nullptr;
    this->d_ = nullptr;

    const char* robot_path = get_okay_robot_xml_path().c_str();
    char errstr[500];

    this->m_ = mj_loadXML(robot_path, NULL, errstr, 500);
    if (!m_) {
        throw std::runtime_error("model not loaded: " + std::string(errstr));
    }
    this->d_ = mj_makeData(m_);

    // spin off gui thread
    this->gui_shutdown_flag_.store(false);
    auto mujoco_gui_thread = std::bind(spin_mujoco_gui, this->m_, this->d_,
        std::ref(this->gui_shutdown_flag_), std::ref(this->mutex_));
    this->gui_thread_ = std::thread(mujoco_gui_thread);

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(this->m_->opt.timestep));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&MujocoSimNode::timer_callback_, this));

    // set up pubs/subs
    if (run_as_puppet) {
        RCLCPP_INFO(this->get_logger(),
            "running in puppet mode, subscribing to servo bus driver observations");

        this->servo_bus_observation_subscriber_
            = this->create_subscription<okay_robot_msgs::msg::ServoBusObservation>(
                TOPIC_SERVO_BUS_OBSERVATION, 10,
                std::bind(&MujocoSimNode::servo_bus_observation_subscriber_callback_, this, _1));
    } else {
        RCLCPP_INFO(this->get_logger(),
            "running in full sim mode, generating servo bus driver observations from mujoco");

        this->servo_bus_observation_publisher_
            = this->create_publisher<okay_robot_msgs::msg::ServoBusObservation>(
                TOPIC_SERVO_BUS_OBSERVATION, 10);
        this->servo_bus_command_subscriber_
            = this->create_subscription<okay_robot_msgs::msg::ServoBusCommand>(
                TOPIC_SERVO_BUS_COMMAND, 10,
                std::bind(&MujocoSimNode::servo_bus_command_subscriber_callback_, this, _1));
    }
}

MujocoSimNode::~MujocoSimNode()
{
    this->gui_shutdown_flag_.store(true);
    this->gui_thread_.join();

    mj_deleteData(this->d_);
    mj_deleteModel(this->m_);
}

void MujocoSimNode::timer_callback_()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    mj_step(this->m_, this->d_);

    if (this->servo_bus_observation_publisher_) {
        this->publish_observations_();
    }

    if (this->gui_shutdown_flag_.load()) {
        rclcpp::shutdown();
    }
}

void MujocoSimNode::publish_observations_()
{
    std::vector<okay_robot_msgs::msg::ServoObservation> observations;
    for (int i = 0; i < this->m_->nu; i++) {
        auto new_observation = okay_robot_msgs::msg::ServoObservation();
        new_observation.id = i + 1;
        new_observation.position = this->d_->qpos[i];
        new_observation.speed = this->d_->qvel[i];

        observations.push_back(new_observation);
    }

    auto servo_bus_observation = okay_robot_msgs::msg::ServoBusObservation();
    servo_bus_observation.observations = observations;

    this->servo_bus_observation_publisher_->publish(servo_bus_observation);
}

void MujocoSimNode::servo_bus_command_subscriber_callback_(
    const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto command : msg->commands) {
        if (command.id > this->m_->nu) {
            RCLCPP_WARN(this->get_logger(), "joint%d out of range: only %d total joints",
                command.id, this->m_->nu);
            continue;
        }

        this->d_->ctrl[command.id - 1] = command.position;
    }
}

void MujocoSimNode::servo_bus_observation_subscriber_callback_(
    const okay_robot_msgs::msg::ServoBusObservation::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    for (auto observation : msg->observations) {
        if (observation.id > this->m_->nu) {
            RCLCPP_WARN(this->get_logger(), "joint%d out of range: only %d total joints",
                observation.id, this->m_->nu);
            continue;
        }

        this->d_->ctrl[observation.id - 1] = observation.position;
    }
}