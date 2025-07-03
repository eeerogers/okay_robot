#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_sim/mujoco_sim_node.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

MujocoSimNode::MujocoSimNode()
    : Node("mujoco_sim_node")
{
    this->m_ = nullptr;
    this->d_ = nullptr;

    // initialize mujoco environment
    const char* robot_path = "/home/eric/projects/mujoco_menagerie/stanford_tidybot/scene.xml";
    char errstr[500];

    this->m_ = mj_loadXML(robot_path, NULL, errstr, 500);
    if (!m_) {
        throw std::runtime_error("model not loaded: " + std::string(errstr));
    }
    this->d_ = mj_makeData(m_);

    // set loop frequency
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(this->m_->opt.timestep));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&MujocoSimNode::timer_callback, this));

    // set up publishers
    this->publisher_ = this->create_publisher<std_msgs::msg::String>("hello_world", 10);
}

MujocoSimNode::~MujocoSimNode()
{
    mj_deleteData(d_);
    mj_deleteModel(m_);
}

void MujocoSimNode::timer_callback()
{
    mj_step(this->m_, this->d_);

    // publish data
    auto message = std_msgs::msg::String();
    message.data = "joint 0: " + std::to_string(this->d_->qpos[0]);
    this->publisher_->publish(message);
}