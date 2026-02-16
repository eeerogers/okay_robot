#pragma once

#include "mujoco/mujoco.h"
#include <GLFW/glfw3.h>
#include <atomic>
#include <mutex>

namespace OkayRobot {
void spinMujocoGUI(mjModel* m, mjData* d, std::atomic<bool>& shutdown_flag, std::mutex& mutex);
class MujocoGUI {
public:
    MujocoGUI(mjModel* m, mjData* d, std::mutex& mutex)
        : m_(m)
        , d_(d)
        , mutex_(mutex) { };
    ~MujocoGUI();

    void init();
    void update();

    bool shouldClose();

private:
    void onMouseButton_(GLFWwindow* window, int button, int act, int mods);
    void onMouseMove_(GLFWwindow* window, double x_pos, double y_pos);
    void onMouseScroll_(GLFWwindow* window, double x_offset, double y_offset);

    static void mouseMoveCallback_(GLFWwindow* window, double x_pos, double y_pos);
    static void mouseButtonCallback_(GLFWwindow* window, int button, int act, int mods);
    static void mouseScrollCallback_(GLFWwindow* window, double x_offset, double y_offset);

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
}