
#include "build_with_robots/mujoco_gui.hpp"
#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

int MujocoGUI::init()
{
    // init glfw
    if (!glfwInit()) {
        printf("error initializing GLFW\n");
        return 0;
    }

    this->window = glfwCreateWindow(1200, 900, "MuJoCo Sim", nullptr, nullptr);
    if (!this->window) {
        printf("failed to create GLFW window\n");
        return 0;
    }

    glfwMakeContextCurrent(this->window);
    glfwSwapInterval(1);

    // set callbacks
    glfwSetWindowUserPointer(this->window, this);
    glfwSetMouseButtonCallback(this->window, MujocoGUI::mouse_button_callback);
    glfwSetCursorPosCallback(this->window, MujocoGUI::mouse_move_callback);
    glfwSetScrollCallback(this->window, MujocoGUI::mouse_scroll_callback);

    // init visualization variables
    mjv_defaultCamera(&this->camera);
    mjv_defaultOption(&this->option);
    mjv_defaultScene(&this->scene);
    mjr_defaultContext(&this->context);

    if (!this->m) {
        printf("mjModel not set before init, cannot proceed\n");
        return 0;
    }

    mjv_makeScene(*this->m, &this->scene, 2000);
    mjr_makeContext(*this->m, &this->context, mjFONTSCALE_150);

    return 1;
}

void MujocoGUI::update()
{
    mjv_updateScene(
        *this->m, *this->d, &this->option, nullptr, &this->camera, mjCAT_ALL, &this->scene);

    int width, height;
    glfwGetFramebufferSize(this->window, &width, &height);

    mjrRect viewport = { 0, 0, width, height };
    mjr_render(viewport, &this->scene, &this->context);

    glfwSwapBuffers(this->window);
    glfwPollEvents();
}

void MujocoGUI::free()
{
    mjv_freeScene(&this->scene);
    mjr_freeContext(&this->context);
    glfwTerminate();
}

bool MujocoGUI::should_close() { return glfwWindowShouldClose(this->window); }

void MujocoGUI::on_mouse_button(GLFWwindow* window, int button, int act, int mods)
{
    this->button_left = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS);
    this->button_right = (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS);

    glfwGetCursorPos(window, &this->last_x, &this->last_y);
}

void MujocoGUI::on_mouse_move(GLFWwindow* window, double xpos, double ypos)
{
    if (!this->button_left && !this->button_right)
        return;

    int width, height;
    glfwGetWindowSize(window, &width, &height);

    // compute mouse displacement
    double dx = xpos - this->last_x;
    double dy = ypos - this->last_y;
    this->last_x = xpos;
    this->last_y = ypos;

    mjtMouse action;
    if (this->button_right)
        action = mjMOUSE_MOVE_H;
    else
        action = mjMOUSE_ROTATE_H;

    mjv_moveCamera(*this->m, action, dx / height, dy / height, &this->scene, &this->camera);
}

void MujocoGUI::on_mouse_scroll(GLFWwindow* window, double xoffset, double yoffset)
{
    mjv_moveCamera(*this->m, mjMOUSE_ZOOM, 0, 0.1 * yoffset, &this->scene, &this->camera);
}

void MujocoGUI::mouse_button_callback(GLFWwindow* window, int button, int act, int mods)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_button(window, button, act, mods);
    }
}

void MujocoGUI::mouse_move_callback(GLFWwindow* window, double xpos, double ypos)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_move(window, xpos, ypos);
    }
}

void MujocoGUI::mouse_scroll_callback(GLFWwindow* window, double xoffset, double yoffset)
{
    MujocoGUI* instance = static_cast<MujocoGUI*>(glfwGetWindowUserPointer(window));
    if (instance) {
        instance->on_mouse_scroll(window, xoffset, yoffset);
    }
}