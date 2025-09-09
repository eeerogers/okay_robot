#include "okay_robot_driver/gamepad/gamepad_node.hpp"
#include "okay_robot_driver/gamepad/gamepad.hpp"
#include "okay_robot_msgs/msg/gamepad_command.hpp"
#include "rclcpp/rclcpp.hpp"
#include <SDL2/SDL.h>
#include <string>

GamepadNode::GamepadNode()
    : Node("gamepad_node")
{
    if (SDL_Init(SDL_INIT_JOYSTICK) < 0) {
        throw std::runtime_error(
            "Unable to initialize SDL joystick: " + std::string(SDL_GetError()));
    }

    if (SDL_NumJoysticks() <= 0) {
        throw std::runtime_error("No gamepads connected");
    }

    this->gamepad_ = SDL_JoystickOpen(0);
    if (this->gamepad_ == NULL) {
        throw std::runtime_error("Unable to open gamepad");
    }

    RCLCPP_INFO(this->get_logger(), "Initialized gamepad");

    this->gamepad_state_ = okay_robot_msgs::msg::GamepadCommand();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->poll_freq_));
    this->timer_ = this->create_wall_timer(duration, std::bind(&GamepadNode::timer_callback, this));
    this->publisher_ = this->create_publisher<okay_robot_msgs::msg::GamepadCommand>("gamepad", 10);

    RCLCPP_INFO(this->get_logger(), "Initialized timer");
}

GamepadNode::~GamepadNode()
{
    SDL_JoystickClose(this->gamepad_);
    SDL_Quit();
}

void GamepadNode::timer_callback()
{
    while (SDL_PollEvent(&this->event_)) {
        switch (this->event_.type) {
        case SDL_JOYAXISMOTION:
            set_axis(&this->gamepad_state_, this->event_.jaxis.axis, this->event_.jaxis.value);
            break;

        case SDL_JOYHATMOTION:
            set_dpad(&this->gamepad_state_, this->event_.jhat.value, true);
            break;

        case SDL_JOYBUTTONDOWN:
            set_button(&this->gamepad_state_, this->event_.jbutton.button, true);
            break;

        case SDL_JOYBUTTONUP:
            set_button(&this->gamepad_state_, this->event_.jbutton.button, false);
            break;

        case SDL_JOYDEVICEADDED:
            break;

        case SDL_JOYDEVICEREMOVED:
            break;
        }
    }

    this->publisher_->publish(this->gamepad_state_);
}
