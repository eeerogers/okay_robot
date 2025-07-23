#pragma once

#include "okay_robot_msgs/msg/gamepad_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <SDL2/SDL.h>

class GamepadNode : public rclcpp::Node {
public:
    GamepadNode();
    ~GamepadNode();

private:
    void timer_callback();

    double poll_freq_ = 100.0;
    SDL_Joystick* gamepad_ = nullptr;
    SDL_Event event_;

    okay_robot_msgs::msg::GamepadCommand gamepad_state_;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<okay_robot_msgs::msg::GamepadCommand>::SharedPtr publisher_;
};