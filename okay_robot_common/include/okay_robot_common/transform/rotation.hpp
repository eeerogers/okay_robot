#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Rotation {
public:
    Rotation(const Eigen::Matrix3f matrix)
        : matrix(matrix) { };
    Rotation(const float& euler_x, const float& euler_y, const float& euler_z)
        : Rotation(this->from_euler(euler_x, euler_y, euler_z)) { };

    const Eigen::Matrix3f matrix;

    const Eigen::Vector3f x() const;
    const Eigen::Vector3f y() const;
    const Eigen::Vector3f z() const;

    static Rotation from_euler(const float& x, const float& y, const float& z);
};
}