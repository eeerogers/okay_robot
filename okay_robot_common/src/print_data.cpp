#include "okay_robot_common/print_data.hpp"

void printMessage(std::vector<uint8_t>& message)
{
    printf("message: [ ");
    for (uint8_t byte : message) {
        printf("0x%.2X ", byte);
    }
    printf("]\n");
}