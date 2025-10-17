#include "okay_robot_driver/servo_bus/servo_bus_node.hpp"
#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "okay_robot_msgs/msg/servo_observation.hpp"
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
    /** TODO: make this better */

    std::vector<uint8_t> data_buffer;
    int servo_position;
    float angle_position;

    std::vector<okay_robot_msgs::msg::ServoObservation> observations;
    std::vector<uint8_t> data({ ServoRegister::CURRENT_POSITION, 0x02 });

    for (uint8_t i = 1; i <= 7; i++) {
        data_buffer = this->servo_bus_.read_data(i, data);

        // need to flip the position because the servo outputs reversed direction
        servo_position = 4095 - ((data_buffer[6] << 8) + data_buffer[5]);
        angle_position = servo_position * (1.0 / this->rad_to_range_);

        auto new_observation = okay_robot_msgs::msg::ServoObservation();
        new_observation.id = i;
        new_observation.position = angle_position;
        new_observation.status = data_buffer[4];

        observations.push_back(new_observation);
    }

    auto servo_bus_observation = okay_robot_msgs::msg::ServoBusObservation();
    servo_bus_observation.observations = observations;

    this->publisher_->publish(servo_bus_observation);
}

void ServoBusNode::command_callback_(const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg)
{
    /** TODO: make this better */

    int full_position;
    uint8_t position_lo;
    uint8_t position_hi;

    // turn commands into packets and send to servo bus
    for (auto command : msg->commands) {
        full_position = command.position * this->rad_to_range_;
        position_lo = full_position & 0xFF;
        position_hi = (full_position >> 8) & 0xFF;

        this->servo_bus_.reg_write_data(
            command.id, { ServoRegister::TARGET_POSITION, position_lo, position_hi });
    }

    this->servo_bus_.execute_reg_write();
}