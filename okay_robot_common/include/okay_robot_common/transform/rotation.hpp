#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Rotation {
public:
    Rotation(const Eigen::Matrix3f matrix)
        : matrix_(matrix) { };
    Rotation(const float& euler_x, const float& euler_y, const float& euler_z)
        : Rotation(this->from_euler(euler_x, euler_y, euler_z)) { };

    inline const Eigen::Matrix3f matrix() const { return this->matrix_; };
    inline const Eigen::Vector3f x() const { return this->matrix_.block<3, 1>(0, 0); };
    inline const Eigen::Vector3f y() const { return this->matrix_.block<3, 1>(0, 1); };
    inline const Eigen::Vector3f z() const { return this->matrix_.block<3, 1>(0, 2); };

    static Rotation from_euler(const float& x, const float& y, const float& z);

private:
    Eigen::Matrix3f matrix_;
};
}