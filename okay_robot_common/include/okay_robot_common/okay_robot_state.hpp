#pragma once

#include <chrono>
#include <vector>

class OkayRobotObservation {
public:
    OkayRobotObservation(const std::chrono::steady_clock::time_point t,
        const std::vector<float> pos, const std::vector<float> vel)
        : time(t)
        , joint_positions(pos)
        , joint_velocities(vel) { };

    const std::chrono::steady_clock::time_point time;

    const std::vector<float> joint_positions;
    const std::vector<float> joint_velocities;
};

class OkayRobotCommand {
public:
    OkayRobotCommand(const std::vector<float> pos)
        : joint_positions(pos) { };

    const std::vector<float> joint_positions;
};

class OkayRobotGoal {
public:
    OkayRobotGoal(const std::vector<float> pos)
        : joint_positions(pos) { };

    const std::vector<float> joint_positions;
};