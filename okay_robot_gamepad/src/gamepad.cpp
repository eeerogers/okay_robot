#include "okay_robot_gamepad/gamepad.hpp"
#include "okay_robot_msgs/msg/gamepad_command.hpp"

namespace OkayRobot {

void set_button(okay_robot_msgs::msg::GamepadCommand* msg, const int& val, const bool& state)
{
    switch (val) {
    case static_cast<int>(gamepad_button::BUTTON_D):
        msg->a_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_R):
        msg->b_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_U):
        msg->y_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_L):
        msg->x_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_L1):
        msg->l1_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_R1):
        msg->r1_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_L2):
        msg->l2_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_R2):
        msg->r2_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_L3):
        msg->l3_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_R3):
        msg->r3_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_SELECT):
        msg->select_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_START):
        msg->start_button = state;
        break;
    case static_cast<int>(gamepad_button::BUTTON_CENTER):
        msg->center_button = state;
        break;
    }
}

void set_axis(okay_robot_msgs::msg::GamepadCommand* msg, const int& val, const int& state)
{
    switch (val) {
    case static_cast<int>(gamepad_axis::AXIS_LSTICK_X):
        msg->left_stick_x_axis = state;
        break;
    case static_cast<int>(gamepad_axis::AXIS_LSTICK_Y):
        msg->left_stick_y_axis = state;
        break;
    case static_cast<int>(gamepad_axis::AXIS_RSTICK_X):
        msg->right_stick_x_axis = state;
        break;
    case static_cast<int>(gamepad_axis::AXIS_RSTICK_Y):
        msg->right_stick_y_axis = state;
        break;
    case static_cast<int>(gamepad_axis::AXIS_LTRIGGER):
        msg->l_trigger = state;
        break;
    case static_cast<int>(gamepad_axis::AXIS_RTRIGGER):
        msg->r_trigger = state;
        break;
    }
}

void set_dpad(okay_robot_msgs::msg::GamepadCommand* msg, const int& val, const bool& state)
{
    switch (val) {
    case static_cast<int>(gamepad_dpad::DPAD_NONE):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_D):
        msg->dpad_down = state;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_DL):
        msg->dpad_down = false;
        msg->dpad_down_left = state;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_DR):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = state;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_U):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = state;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_UL):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = state;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_UR):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = state;
        msg->dpad_right = false;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_R):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = state;
        msg->dpad_left = false;
        break;
    case static_cast<int>(gamepad_dpad::DPAD_L):
        msg->dpad_down = false;
        msg->dpad_down_left = false;
        msg->dpad_down_right = false;
        msg->dpad_up = false;
        msg->dpad_up_left = false;
        msg->dpad_up_right = false;
        msg->dpad_right = false;
        msg->dpad_left = state;
        break;
    }
}

}