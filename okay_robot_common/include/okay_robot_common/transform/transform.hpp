#pragma once

#include <Eigen/Dense>

#include "okay_robot_common/transform/denavit_hartenberg.hpp"
#include "okay_robot_common/transform/position.hpp"
#include "okay_robot_common/transform/rotation.hpp"

namespace OkayRobot {
class Transform {
public:
    Transform()
        : matrix(Eigen::Matrix4f::Identity())
        , position(matrix.block<3, 1>(0, 3))
        , rotation(matrix.block<3, 3>(0, 0)) { };
    Transform(const Eigen::Matrix4f& matrix)
        : matrix(matrix)
        , position(matrix.block<3, 1>(0, 3))
        , rotation(matrix.block<3, 3>(0, 0)) { };
    Transform(const Position position, const Rotation rotation)
        : Transform(this->from_position_rotation(position, rotation)) { };
    Transform(const DenavitHartenberg& dh, const float& theta)
        : Transform(this->from_dh(dh, theta)) { };

    const Eigen::Matrix4f matrix;
    const Position position;
    const Rotation rotation;

    Transform forward(const Transform& other) const;
    Transform inverse() const;

    static Transform from_dh(const DenavitHartenberg& dh, const float& theta);
    static Transform from_position_rotation(const Position& position, const Rotation& rotation);
};
}