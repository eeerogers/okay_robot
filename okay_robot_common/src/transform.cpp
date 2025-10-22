#include "okay_robot_common/transform.hpp"

namespace OkayRobot {
Eigen::Vector3f Transform::position() const { return this->matrix.block<3, 1>(0, 3); }

Eigen::Matrix3f Transform::orientation() const { return this->matrix.block<3, 3>(0, 0); }

Transform Transform::forward(const Transform& other) const
{
    return Transform(this->matrix * other.matrix);
}

Transform Transform::inverse() const { return Transform(this->matrix.inverse()); }
}