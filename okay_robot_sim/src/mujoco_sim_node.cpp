#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "okay_robot_msgs/msg/tidy_bot_cmd.hpp"
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
    const char* robot_path = "/home/eric/projects/mujoco_menagerie/stanford_tidybot/scene.xml";
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
    this->tidy_bot_subscriber_ = this->create_subscription<okay_robot_msgs::msg::TidyBotCmd>(
        "tidy_bot_cmd", 10, std::bind(&MujocoSimNode::tidy_bot_subscriber_callback, this, _1));
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
}

void MujocoSimNode::tidy_bot_subscriber_callback(
    const okay_robot_msgs::msg::TidyBotCmd::SharedPtr msg)
{
    // TODO: make this less shitty, obviously
    mjtNum new_base_ctrl[]
        = { msg->x_pos, msg->y_pos, msg->theta, msg->joints[0], msg->joints[1], msg->joints[2],
              msg->joints[3], msg->joints[4], msg->joints[5], msg->joints[6], msg->gripper };

    RCLCPP_INFO(this->get_logger(), "j2: %f", new_base_ctrl[4]);

    std::lock_guard<std::mutex> lock(this->mutex_);
    std::copy(new_base_ctrl, new_base_ctrl + 11, this->d_->ctrl);
}