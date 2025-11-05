#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Transform {
public:
    Transform()
        : matrix(Eigen::Matrix4f::Identity())
        , x(0.0)
        , y(0.0)
        , z(0.0) { };
    Transform(const Eigen::Matrix4f& mat)
        : matrix(mat)
        , x(this->matrix(0, 3))
        , y(this->matrix(1, 3))
        , z(this->matrix(2, 3)) { };
    Transform(const Eigen::Vector3f& position, const Eigen::Matrix3f& rotation)
        : matrix(this->from_position_rotation_(position, rotation))
        , x(this->matrix(0, 3))
        , y(this->matrix(1, 3))
        , z(this->matrix(2, 3)) { };

    const Eigen::Matrix4f matrix;
    const float x;
    const float y;
    const float z;

    Eigen::Vector3f position() const;
    Eigen::Matrix3f rotation() const;

    Transform forward(const Transform& other) const;
    Transform inverse() const;

private:
    static Eigen::Matrix4f from_position_rotation_(
        const Eigen::Vector3f& position, const Eigen::Matrix3f& rotation);
};

Eigen::Matrix3f euler_to_rotation(const float& x, const float& y, const float& z);
}