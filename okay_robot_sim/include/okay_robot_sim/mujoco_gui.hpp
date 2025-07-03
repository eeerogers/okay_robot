#pragma once

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>

class MujocoGUI {
public:
    MujocoGUI(mjModel** m, mjData** d)
        : m(m)
        , d(d) { };

    int init();
    void update();
    void free();

    bool should_close();

private:
    void on_mouse_button(GLFWwindow* window, int button, int act, int mods);
    void on_mouse_move(GLFWwindow* window, double x_pos, double y_pos);
    void on_mouse_scroll(GLFWwindow* window, double x_offset, double y_offset);

    static void mouse_move_callback(GLFWwindow* window, double x_pos, double y_pos);
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    static void mouse_scroll_callback(GLFWwindow* window, double x_offset, double y_offset);

    mjModel** m;
    mjData** d;

    mjvScene scene;
    mjvCamera camera;
    mjvOption option;
    mjrContext context;
    GLFWwindow* window;

    int button_left;
    int button_right;
    double last_x;
    double last_y;
};