
#include "okay_robot_sim/mujoco_gui.hpp"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <atomic>
#include <functional>
#include <mutex>
#include <stdexcept>
#include <thread>

namespace OkayRobot {

void spinMujocoGUI(mjModel* m, mjData* d, std::atomic<bool>& shutdown_flag, std::mutex& mutex)
{
    MujocoGUI mujoco_gui(m, d, mutex);
    mujoco_gui.init();

    while (!(mujoco_gui.shouldClose() || shutdown_flag.load())) {
        mujoco_gui.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }

    // set flag to true to signal that the gui has shut down
    shutdown_flag.store(true);
}

MujocoGUI::~MujocoGUI()
{
    mjv_freeScene(&this->scene_);
    mjr_freeContext(&this->context_);
    glfwTerminate();
}

void MujocoGUI::init()
{
    // init glfw
    if (!glfwInit()) {
        throw std::runtime_error("error initializing GLFW");
    }

    this->window_ = glfwCreateWindow(1200, 900, "MuJoCo Sim", nullptr, nullptr);
    if (!this->window_) {
        throw std::runtime_error("failed to create GLFW window");
    }

    glfwMakeContextCurrent(this->window_);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetWindowUserPointer(this->window_, this);
    glfwSetMouseButtonCallback(this->window_, MujocoGUI::mouseButtonCallback_);
    glfwSetCursorPosCallback(this->window_, MujocoGUI::mouseMoveCallback_);
    glfwSetScrollCallback(this->window_, MujocoGUI::mouseScrollCallback_);

    // init visualization variables
    mjv_defaultCamera(&this->camera_);
    mjv_defaultOption(&this->option_);
    mjv_defaultScene(&this->scene_);
    mjr_defaultContext(&this->context_);

    if (!this->m_) {
        throw std::runtime_error("mjModel not set before init, cannot proceed");
    }

    mjv_makeScene(this->m_, &this->scene_, 2000);
    mjr_makeContext(this->m_, &this->context_, mjFONTSCALE_150);
}

void MujocoGUI::update()
{
    std::lock_guard<std::mutex> lock(this->mutex_);
    mjv_updateScene(
        this->m_, this->d_, &this->option_, nullptr, &this->camera_, mjCAT_ALL, &this->scene_);

    int width, height;
    glfwGetFramebufferSize(this->window_, &width, &height);

    mjrRect viewport = { 0, 0, width, height };
    mjr_render(viewport, &this->scene_, &this->context_);

    glfwSwapBuffers(this->window_);
    glfwPollEvents();
}

bool MujocoGUI::shouldClose() { return glfwWindowShouldClose(this->window_); }

void MujocoGUI::onMouseButton_(GLFWwindow* window, int button, int act, int mods)
{
    this->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    this->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &this->last_x_, &this->last_y_);
}

void MujocoGUI::onMouseMove_(GLFWwindow* window, double x_pos, double ypos)
{
    if (!this->button_left_ && !this->button_right_)
        return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // compute mouse displacement
    double dx = x_pos - this->last_x_;
    double dy = ypos - this->last_y_;
    this->last_x_ = x_pos;
    this->last_y_ = ypos;

    mjtMouse action;
    if (this->button_right_)
        action = mjMOUSE_MOVE_H;
    else
        action = mjMOUSE_ROTATE_H;

    mjv_moveCamera(this->m_, action, dx / height, dy / height, &this->scene_, &this->camera_);
}

void MujocoGUI::onMouseScroll_(GLFWwindow* window, double x_offset, double y_offset)
{
    mjv_moveCamera(this->m_, mjMOUSE_ZOOM, 0, 0.1 * y_offset, &this->scene_, &this->camera_);
}

void MujocoGUI::mouseButtonCallback_(GLFWwindow* window, int button, int act, int mods)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->onMouseButton_(window, button, act, mods);
    }
}

void MujocoGUI::mouseMoveCallback_(GLFWwindow* window, double x_pos, double y_pos)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->onMouseMove_(window, x_pos, y_pos);
    }
}

void MujocoGUI::mouseScrollCallback_(GLFWwindow* window, double x_offset, double y_offset)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->onMouseScroll_(window, x_offset, y_offset);
    }
}

}