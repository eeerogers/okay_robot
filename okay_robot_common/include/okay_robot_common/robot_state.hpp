#pragma once

#include <chrono>
#include <vector>

namespace OkayRobot {
class Observation {
public:
    Observation(const std::chrono::steady_clock::time_point t, const std::vector<float> pos,
        const std::vector<float> vel)
        : time(t)
        , joint_positions(pos)
        , joint_velocities(vel) { };

    const std::chrono::steady_clock::time_point time;

    const std::vector<float> joint_positions;
    const std::vector<float> joint_velocities;
};

class Command {
public:
    Command(const std::vector<float> pos)
        : joint_positions(pos) { };

    const std::vector<float> joint_positions;
};

class Pose {
public:
    Pose(const std::vector<float> pos)
        : joint_positions(pos) { };

    const std::vector<float> joint_positions;
};

bool pose_is_valid(const Pose pose);
}