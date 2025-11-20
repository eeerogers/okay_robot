#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <vector>

#include "okay_robot_common/transform.hpp"

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
    Pose(const Eigen::Vector<float, 6> pos)
        : joint_positions(std::vector<float>(pos.data(), pos.data() + pos.size())) { };

    const std::vector<float> joint_positions;
};

class GamepadState {
public:
    GamepadState(const Transform& transform, const float& eef_position)
        : transform(transform)
        , eef_position(eef_position) { };

    Eigen::Vector3f position() { return this->transform.position(); };
    Eigen::Matrix3f rotation() { return this->transform.rotation(); };

    const Transform transform;
    const float eef_position;
};

Eigen::Vector<float, 6> pose_to_eigen_vector(const Pose pose);
bool pose_is_valid(const Pose pose);
}