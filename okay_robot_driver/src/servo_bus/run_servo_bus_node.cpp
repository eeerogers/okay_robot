#include "okay_robot_driver/servo_bus/servo_bus.hpp"
#include "rclcpp/rclcpp.hpp"

int main(int argc, char* argv[])
{
    // TODO: implement
    // rclcpp::init(argc, argv);
    // rclcpp::spin(std::make_shared<ServoBusNode>());
    // rclcpp::shutdown();

    ServoBus servo_bus;
    servo_bus.init("/dev/ttyACM0", LibSerial::BaudRate::BAUD_1000000);
    servo_bus.ping(0x01);
    print_message(servo_bus.read_message());

    printf("donezo, cya\n");

    return 0;
}