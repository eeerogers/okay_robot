#include <cmath>

#include "okay_robot_common/print_data.hpp"
#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "okay_robot_driver/servo_bus/servo_bus_node.hpp"
#include "okay_robot_msgs/msg/servo_bus_command.hpp"
#include "okay_robot_msgs/msg/servo_bus_observation.hpp"
#include "okay_robot_msgs/msg/servo_observation.hpp"

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
    this->publish_observation();
    this->execute_next_command_();
}

void ServoBusNode::publish_observation()
{
    /** TODO: make this better */

    std::vector<uint8_t> data_buffer;
    int servo_position;
    int servo_speed;
    int servo_load;
    int servo_voltage;
    int servo_current;

    float servo_position_rad;
    float servo_speed_rad_s;
    float servo_load_pct;
    float servo_voltage_v;
    float servo_current_a;

    // position: 2 bytes
    // speed: 2 bytes
    // load: 2 bytes
    // operating voltage: 1 byte
    // temperature: 1 byte
    // async write flag: 1 byte
    // servo status: 1 byte
    // move flag: 1 byte
    // also position?: 2 bytes
    // current: 2 bytes
    std::vector<okay_robot_msgs::msg::ServoObservation> observations;
    std::vector<uint8_t> data({ ServoRegister::CURRENT_POSITION, 0x0F });

    for (uint8_t i = 1; i <= 7; i++) {
        data_buffer = this->servo_bus_.read_data(i, data);

        // TODO: handle this more gracefully?
        // return without publishing if anything read is bad
        if (data_buffer.empty() || (uint8_t)data_buffer[2] != i)
            return;

        // need to flip the position because the servo outputs reversed direction
        servo_position = 4095 - ((data_buffer[6] << 8) + data_buffer[5]);
        servo_speed = ((data_buffer[8] << 8) | data_buffer[7]);
        servo_load = ((data_buffer[10] << 8) | data_buffer[9]);
        servo_voltage = data_buffer[11];
        servo_current = ((data_buffer[19] << 8) | data_buffer[18]);

        // convert signed values
        servo_position
            = servo_position & (1 << 15) ? -(servo_position & ~(1 << 15)) : servo_position;
        servo_speed = servo_speed & (1 << 15) ? -(servo_speed & ~(1 << 15)) : servo_speed;
        servo_load = servo_load & (1 << 10) ? -(servo_load & ~(1 << 10)) : servo_load;
        servo_current = servo_current & (1 << 15) ? -(servo_current & (1 << 15)) : servo_current;

        // map to units
        servo_position_rad = servo_position * (1.0 / this->rad_to_range_);
        servo_speed_rad_s = servo_speed * (1.0 / this->rad_to_range_);
        servo_load_pct = servo_load * 0.1;
        servo_voltage_v = servo_voltage * 0.1;
        servo_current_a = servo_current * 0.0065;

        // build ros message
        auto new_observation = okay_robot_msgs::msg::ServoObservation();
        new_observation.id = data_buffer[2];
        new_observation.position = servo_position_rad;
        new_observation.speed = servo_speed_rad_s;
        new_observation.load = servo_load_pct;
        new_observation.voltage = servo_voltage_v;
        new_observation.temperature = data_buffer[12];
        new_observation.current = servo_current_a;
        new_observation.is_moving = data_buffer[15];
        new_observation.status = data_buffer[4];

        observations.push_back(new_observation);
    }

    auto servo_bus_observation = okay_robot_msgs::msg::ServoBusObservation();
    servo_bus_observation.observations = observations;

    this->publisher_->publish(servo_bus_observation);
}

void ServoBusNode::execute_next_command_()
{
    /** TODO: make this better */

    if (this->command_queue_.empty())
        return;

    auto next_command = this->command_queue_.front();
    this->command_queue_.pop();

    int full_position;
    uint8_t position_lo;
    uint8_t position_hi;

    // turn commands into packets and send to servo bus
    for (auto command : next_command.commands) {
        full_position = 4095 - (command.position * this->rad_to_range_);
        position_lo = full_position & 0xFF;
        position_hi = (full_position >> 8) & 0xFF;

        this->servo_bus_.reg_write_data(
            command.id, { ServoRegister::TARGET_POSITION, position_lo, position_hi });
    }

    this->servo_bus_.execute_reg_write();
}

void ServoBusNode::command_callback_(const okay_robot_msgs::msg::ServoBusCommand::SharedPtr msg)
{
    this->command_queue_.push(*msg.get());
}