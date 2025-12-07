#include "okay_robot_common/transform/transform.hpp"

namespace OkayRobot {
Transform Transform::forward(const Transform& other) const
{
    return Transform(this->matrix * other.matrix);
}

Transform Transform::inverse() const { return Transform(this->matrix.inverse()); }

Transform Transform::from_dh(const DenavitHartenberg& dh, const float& theta)
{
    const float new_theta = dh.theta + theta;

    const float sin_theta = std::sin(new_theta);
    const float cos_theta = std::cos(new_theta);
    const float sin_alpha = std::sin(dh.alpha);
    const float cos_alpha = std::cos(dh.alpha);

    /** TODO: find a cleaner way to format this? */
    Eigen::Matrix4f m;
    m << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, cos_theta * dh.a, sin_theta,
        cos_theta * cos_alpha, -cos_theta * sin_alpha, sin_theta * dh.a, 0.0, sin_alpha, cos_alpha,
        dh.d, 0.0, 0.0, 0.0, 1.0;

    return Transform(m);
}

Transform Transform::from_position_rotation(const Position& position, const Rotation& rotation)
{
    Eigen::Matrix4f matrix = Eigen::Matrix4f::Identity();

    matrix.block(0, 0, 3, 3) = rotation.matrix;
    matrix.block(0, 3, 3, 1) = position.vector;

    return Transform(matrix);
}
}