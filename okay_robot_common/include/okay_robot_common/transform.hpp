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
        , x(mat(0, 3))
        , y(mat(1, 3))
        , z(mat(2, 3)) { };

    const Eigen::Matrix4f matrix;
    const float x;
    const float y;
    const float z;

    Eigen::Vector3f position() const;
    Eigen::Matrix3f orientation() const;

    Transform forward(const Transform& other) const;
    Transform inverse() const;
};
}