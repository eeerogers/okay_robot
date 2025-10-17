#include <numeric>

#include "okay_robot_common/print_data.hpp"

void print_message(std::vector<uint8_t>& message)
{
    printf("message: [ ");
    for (uint8_t byte : message) {
        printf("0x%.2X ", byte);
    }
    printf("]\n");
}

template <typename T> std::string vec_to_string(std::vector<T>& vec)
{
    std::string as_string = std::accumulate(vec.begin(), vec.end(), std::string(),
        [](std::string& s, const T& n) { return s + (s.empty() ? "" : " ") + std::to_string(n); });

    return as_string;
}