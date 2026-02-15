#include "okay_robot_common/transform/position.hpp"

namespace OkayRobot {
Eigen::Vector3f Position::from_floats(const float& x, const float& y, const float& z)
{
    Eigen::Vector3f position_vector;
    position_vector << x, y, z;

    return position_vector;
}
}