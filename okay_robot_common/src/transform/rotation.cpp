#include "okay_robot_common/transform/rotation.hpp"

namespace OkayRobot {

Rotation Rotation::fromEuler(const float& x, const float& y, const float& z)
{
    // just xyz for now
    // TODO: add toggle for different orders of angles

    Eigen::Matrix3f r_x;
    float sinx = std::sin(x);
    float cosx = std::cos(x);
    r_x << 1.0, 0.0, 0.0, 0.0, cosx, -sinx, 0.0, sinx, cosx;

    Eigen::Matrix3f r_y;
    float siny = std::sin(y);
    float cosy = std::cos(y);
    r_y << cosy, 0.0, siny, 0.0, 1.0, 0.0, -siny, 0.0, cosy;

    Eigen::Matrix3f r_z;
    float sinz = std::sin(z);
    float cosz = std::cos(z);
    r_z << cosz, -sinz, 0.0, sinz, cosz, 0.0, 0.0, 0.0, 1.0;

    return Rotation(r_x * r_y * r_z);
}
}