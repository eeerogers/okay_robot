#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Position {
public:
    Position(const Eigen::Vector3f vector)
        : vector(vector) { };
    Position(const float& x, const float& y, const float& z)
        : Position(this->from_floats(x, y, z)) { };

    const Eigen::Vector3f vector;

    const float x() const;
    const float y() const;
    const float z() const;

private:
    static Eigen::Vector3f from_floats(const float& x, const float& y, const float& z);
};
}