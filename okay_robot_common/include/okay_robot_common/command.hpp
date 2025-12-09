#pragma once

#include <vector>

namespace OkayRobot {
class JointPositionCommand {
public:
    JointPositionCommand(const std::vector<float> pos)
        : joint_positions(pos) { };

    const std::vector<float> joint_positions;
};

class JointVelocityCommand {
public:
    JointVelocityCommand(const std::vector<float> vel)
        : joint_velocities(vel) { };

    const std::vector<float> joint_velocities;
};
}