#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_description/descriptions.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_sim/mujoco_gui.hpp"
#include "okay_robot_sim/mujoco_sim_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;

MujocoSimNode::MujocoSimNode()
    : Node("mujoco_sim_node")
{
    this->m_ = nullptr;
    this->d_ = nullptr;

    // initialize mujoco environment
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
        = this->create_wall_timer(duration, std::bind(&MujocoSimNode::timer_callback, this));

    // set up pubs/subs
    this->publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
    this->servo_bus_command_subscriber_
        = this->create_subscription<okay_robot_msgs::msg::ServoBusCommand>("servo_bus_command", 10,
            std::bind(&MujocoSimNode::servo_bus_subscriber_callback, this, _1));
}

MujocoSimNode::~MujocoSimNode()
{
    this->gui_shutdown_flag_.store(true);
    this->gui_thread_.join();

    mj_deleteData(this->d_);
    mj_deleteModel(this->m_);
}

void MujocoSimNode::timer_callback()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    mj_step(this->m_, this->d_);

    // publish data
    auto message = std_msgs::msg::String();
    message.data = "joint 0: " + std::to_string(this->d_->qpos[0]);
    this->publisher_->publish(message);

    if (this->gui_shutdown_flag_.load()) {
        std::exit(0);
    }
}

void MujocoSimNode::servo_bus_subscriber_callback(
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
        RCLCPP_INFO(this->get_logger(), "joint%d: %f", command.id, this->d_->ctrl[command.id - 1]);
    }
}