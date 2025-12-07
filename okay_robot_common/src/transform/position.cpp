#include "okay_robot_common/transform/position.hpp"

namespace OkayRobot {
const float Position::x() const { return this->vector[0]; }

const float Position::y() const { return this->vector[1]; }

const float Position::z() const { return this->vector[2]; }

Eigen::Vector3f Position::from_floats(const float& x, const float& y, const float& z)
{
    Eigen::Vector3f position_vector;
    position_vector << x, y, z;

    return position_vector;
}
}