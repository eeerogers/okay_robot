#pragma once

#include "okay_robot_common/transform/transform.hpp"

namespace OkayRobot {
class GamepadState {
public:
    GamepadState(const Transform& transform, const float& eef_position)
        : transform(transform)
        , eef_position(eef_position) { };

    Position position() { return this->transform.position; };
    Rotation rotation() { return this->transform.rotation; };

    const Transform transform;
    const float eef_position;
};
}