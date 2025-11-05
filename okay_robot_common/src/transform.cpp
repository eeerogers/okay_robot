#include "okay_robot_common/transform.hpp"

namespace OkayRobot {
Eigen::Vector3f Transform::position() const { return this->matrix.block<3, 1>(0, 3); }

Eigen::Matrix3f Transform::rotation() const { return this->matrix.block<3, 3>(0, 0); }

Transform Transform::forward(const Transform& other) const
{
    return Transform(this->matrix * other.matrix);
}

Transform Transform::inverse() const { return Transform(this->matrix.inverse()); }

Eigen::Matrix4f Transform::from_position_rotation_(
    const Eigen::Vector3f& position, const Eigen::Matrix3f& rotation)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

    matrix.block(0, 0, 3, 3) = rotation;
    matrix.block(0, 3, 3, 1) = position;

    return matrix;
}

Eigen::Matrix3f euler_to_rotation(const float& x, const float& y, const float& z)
{
    // just xyz for now
    // TODO: add toggle for different orders of angles

    Eigen::Matrix3f r_x;
    float sinx = std::sin(x);
    float cosx = std::cos(x);
    r_x << 1.0, 0.0, 0.0, 0.0, cosx, -sinx, 0.0, sinx, cosx;

    Eigen::Matrix3f r_y;
    float siny = std::sin(y);
    float cosy = std::cos(y);
    r_y << cosy, 0.0, siny, 0.0, 1.0, 0.0, -siny, 0.0, cosy;

    Eigen::Matrix3f r_z;
    float sinz = std::sin(z);
    float cosz = std::cos(z);
    r_z << cosz, -sinz, 0.0, sinz, cosz, 0.0, 0.0, 0.0, 1.0;

    return r_x * r_y * r_z;
}
}