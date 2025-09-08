#include "okay_robot_driver/servo_bus_node.hpp"
#include "okay_robot_driver/servo_bus.hpp"

ServoBusNode::ServoBusNode()
    : Node("servo_bus_node")
{
    this->servo_bus_.init(this->port_, this->baud_);

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(1.0 / this->poll_freq_));
    this->timer_
        = this->create_wall_timer(duration, std::bind(&ServoBusNode::timer_callback_, this));
}

// TODO: implement
void ServoBusNode::timer_callback_()
{
    //
}