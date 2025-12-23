#pragma once

#include <memory>
#include <unordered_map>
#include <vector>

#include "okay_robot_msgs/msg/servo_bus_observation.hpp"

class TimeSeriesData {
public:
    TimeSeriesData(
        std::vector<std::shared_ptr<okay_robot_msgs::msg::ServoBusObservation>> observations);

    std::vector<float> time;
    std::unordered_map<int, std::vector<float>> positions;
    std::unordered_map<int, std::vector<float>> speeds;
    std::unordered_map<int, std::vector<float>> loads;
    std::unordered_map<int, std::vector<float>> voltages;
    std::unordered_map<int, std::vector<float>> temperatures;
    std::unordered_map<int, std::vector<float>> currents;
};