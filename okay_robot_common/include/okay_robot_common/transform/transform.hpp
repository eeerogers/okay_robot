#pragma once

#include <Eigen/Dense>

#include "okay_robot_common/transform/denavit_hartenberg.hpp"
#include "okay_robot_common/transform/position.hpp"
#include "okay_robot_common/transform/rotation.hpp"

namespace OkayRobot {
class Transform {
public:
    Transform()
        : matrix_(Eigen::Matrix4f::Identity())
        , position_(matrix_.block<3, 1>(0, 3))
        , rotation_(matrix_.block<3, 3>(0, 0)) { };
    Transform(const Eigen::Matrix4f& matrix)
        : matrix_(matrix)
        , position_(matrix.block<3, 1>(0, 3))
        , rotation_(matrix.block<3, 3>(0, 0)) { };
    Transform(const Position position, const Rotation rotation)
        : Transform(this->fromPositionRotation(position, rotation)) { };
    Transform(const DenavitHartenberg& dh, const float& theta)
        : Transform(this->fromDH(dh, theta)) { };

    void operator=(const Transform& other);

    inline const Eigen::Matrix4f matrix() const { return this->matrix_; };
    inline const Position position() const { return this->position_; };
    inline const Rotation rotation() const { return this->rotation_; };

    Transform forward(const Transform& other) const;
    Transform inverse() const;

    static Transform fromDH(const DenavitHartenberg& dh, const float& theta);
    static Transform fromPositionRotation(const Position& position, const Rotation& rotation);

private:
    Eigen::Matrix4f matrix_;
    Position position_;
    Rotation rotation_;
};
}