#pragma once

#include "okay_robot_msgs/msg/gamepad_command.hpp"

enum gamepad_button {
    BUTTON_D = 0,
    BUTTON_R = 1,
    BUTTON_U = 2,
    BUTTON_L = 3,

    BUTTON_L1 = 4,
    BUTTON_R1 = 5,
    BUTTON_L2 = 6,
    BUTTON_R2 = 7,

    BUTTON_SELECT = 8,
    BUTTON_START = 9,
    BUTTON_CENTER = 10,

    BUTTON_L3 = 11,
    BUTTON_R3 = 12,
};

enum gamepad_axis {
    AXIS_LSTICK_X = 0,
    AXIS_LSTICK_Y = 1,
    AXIS_LTRIGGER = 2,

    AXIS_RSTICK_X = 3,
    AXIS_RSTICK_Y = 4,
    AXIS_RTRIGGER = 5,
};

enum gamepad_dpad {
    DPAD_NONE = 0,
    DPAD_U = 1,
    DPAD_UR = 3,
    DPAD_D = 4,
    DPAD_DL = 12,
    DPAD_R = 2,
    DPAD_RD = 6,
    DPAD_L = 8,
    DPAD_UL = 9,
};

void set_button(okay_robot_msgs::msg::GamepadCommand* msg, int val, bool state);
void set_axis(okay_robot_msgs::msg::GamepadCommand* msg, int val, float state);
void set_dpad(okay_robot_msgs::msg::GamepadCommand* msg, int val, bool state);