#include "okay_robot_driver/servo_bus/servo_bus_node.hpp"
#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include <cmath>

using std::placeholders::_1;

ServoBusNode::ServoBusNode()
    : Node("servo_bus_node")
{
    this->servo_bus_.init(this->port_, this->baud_);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->poll_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&ServoBusNode::timer_callback_, this));

    this->publisher_ = this->create_publisher<okay_robot_msgs::msg::ServoBusObservation>(
        "servo_bus_observation", 10);
    this->subscriber_ = this->create_subscription<okay_robot_msgs::msg::ServoBusCommand>(
        "servo_bus_command", 10, std::bind(&ServoBusNode::command_callback_, this, _1));
}

void ServoBusNode::timer_callback_()
{
    /** TODO: implement */
    // send BULK_READ for Position, Speed, Load, Voltage, Temperature, Current, Is-Moving, Status
}

void ServoBusNode::command_callback_(const okay_robot_msgs::msg::ServoBusCommand msg)
{
    /** TODO: make this all more efficient */

    int full_position;
    uint8_t position_lo;
    uint8_t position_hi;

    // servo range 0..4095
    float conversion_factor = 4095.0 / (2.0 * M_PI);

    // turn commands into packets and send to servo bus
    for (auto command : msg.commands) {
        full_position = command.position * conversion_factor;
        position_lo = full_position & 0xFF;
        position_hi = (full_position >> 8) & 0xFF;

        this->servo_bus_.write_data(
            command.id, { ServoRegister::TARGET_POSITION, position_lo, position_hi });
    }
}