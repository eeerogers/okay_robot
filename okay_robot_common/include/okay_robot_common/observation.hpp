#pragma once

#include <chrono>
#include <vector>

namespace OkayRobot {
struct Observation {
    const std::chrono::steady_clock::time_point time;
    const std::vector<float> joint_positions;
    const std::vector<float> joint_velocities;
    const std::vector<float> joint_loads;
};
}