
#include "okay_robot_sim/mujoco_gui.hpp"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <mutex>
#include <thread>

void spin_mujoco_gui(mjModel* m, mjData* d, std::mutex* mutex)
{
    MujocoGUI mujoco_gui(m, d, *mutex);
    if (!mujoco_gui.init()) {
        printf("error initializing mujoco gui\n");
        return;
    }

    while (!mujoco_gui.should_close()) {
        mujoco_gui.update();
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
}

MujocoGUI::~MujocoGUI()
{
    mjv_freeScene(&this->scene_);
    mjr_freeContext(&this->context_);
    glfwTerminate();
}

int MujocoGUI::init()
{
    // init glfw
    if (!glfwInit()) {
        printf("error initializing GLFW\n");
        return 0;
    }

    this->window_ = glfwCreateWindow(1200, 900, "MuJoCo Sim", nullptr, nullptr);
    if (!this->window_) {
        printf("failed to create GLFW window\n");
        return 0;
    }

    glfwMakeContextCurrent(this->window_);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetWindowUserPointer(this->window_, this);
    glfwSetMouseButtonCallback(this->window_, MujocoGUI::mouse_button_callback);
    glfwSetCursorPosCallback(this->window_, MujocoGUI::mouse_move_callback);
    glfwSetScrollCallback(this->window_, MujocoGUI::mouse_scroll_callback);

    // init visualization variables
    mjv_defaultCamera(&this->camera_);
    mjv_defaultOption(&this->option_);
    mjv_defaultScene(&this->scene_);
    mjr_defaultContext(&this->context_);

    if (!this->m_) {
        printf("mjModel not set before init, cannot proceed\n");
        return 0;
    }

    mjv_makeScene(this->m_, &this->scene_, 2000);
    mjr_makeContext(this->m_, &this->context_, mjFONTSCALE_150);

    return 1;
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

bool MujocoGUI::should_close() { return glfwWindowShouldClose(this->window_); }

void MujocoGUI::on_mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    this->button_left_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    this->button_right_ = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &this->last_x_, &this->last_y_);
}

void MujocoGUI::on_mouse_move(GLFWwindow* window, double x_pos, double ypos)
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

void MujocoGUI::on_mouse_scroll(GLFWwindow* window, double x_offset, double y_offset)
{
    mjv_moveCamera(this->m_, mjMOUSE_ZOOM, 0, 0.1 * y_offset, &this->scene_, &this->camera_);
}

void MujocoGUI::mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_button(window, button, act, mods);
    }
}

void MujocoGUI::mouse_move_callback(GLFWwindow* window, double x_pos, double y_pos)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_move(window, x_pos, y_pos);
    }
}

void MujocoGUI::mouse_scroll_callback(GLFWwindow* window, double x_offset, double y_offset)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_scroll(window, x_offset, y_offset);
    }
}