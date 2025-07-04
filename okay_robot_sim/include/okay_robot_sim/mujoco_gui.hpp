#pragma once

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <atomic>
#include <mutex>

void spin_mujoco_gui(mjModel* m, mjData* d, std::atomic<bool>& shutdown_flag, std::mutex& mutex);
class MujocoGUI {
public:
    MujocoGUI(mjModel* m, mjData* d, std::mutex& mutex)
        : m_(m)
        , d_(d)
        , mutex_(mutex) { };
    ~MujocoGUI();

    void init();
    void update();

    bool should_close();

private:
    void on_mouse_button(GLFWwindow* window, int button, int act, int mods);
    void on_mouse_move(GLFWwindow* window, double x_pos, double y_pos);
    void on_mouse_scroll(GLFWwindow* window, double x_offset, double y_offset);

    static void mouse_move_callback(GLFWwindow* window, double x_pos, double y_pos);
    static void mouse_button_callback(GLFWwindow* window, int button, int act, int mods);
    static void mouse_scroll_callback(GLFWwindow* window, double x_offset, double y_offset);

    mjModel* m_;
    mjData* d_;
    std::mutex& mutex_;

    mjvScene scene_;
    mjvCamera camera_;
    mjvOption option_;
    mjrContext context_;
    GLFWwindow* window_;

    int button_left_ = 0;
    int button_right_ = 0;
    double last_x_ = 0;
    double last_y_ = 0;
};