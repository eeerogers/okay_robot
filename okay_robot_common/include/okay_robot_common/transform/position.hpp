#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Position {
public:
    Position(const Eigen::Vector3f vector)
        : vector_(vector) { };
    Position(const float& x, const float& y, const float& z)
        : Position(this->from_floats(x, y, z)) { };

    inline const Eigen::Vector3f vector() const { return this->vector_; };
    inline float x() const { return this->vector_[0]; };
    inline float y() const { return this->vector_[1]; };
    inline float z() const { return this->vector_[2]; };

    static Eigen::Vector3f from_floats(const float& x, const float& y, const float& z);

private:
    Eigen::Vector3f vector_;
};
}