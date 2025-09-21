#include "okay_robot_common/transform.hpp"
#include <Eigen/Dense>

Transform Transform::forward(const Transform& other)
{
    return Transform(this->matrix * other.matrix);
}

Transform Transform::inverse() { return Transform(this->matrix.inverse()); }