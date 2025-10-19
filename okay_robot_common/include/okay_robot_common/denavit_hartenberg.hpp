#pragma once

#include "okay_robot_common/transform.hpp"
#include <Eigen/Dense>

namespace OkayRobot {
struct DH {
    const float a;
    const float d;
    const float alpha;
    const float theta;
};

OkayRobot::Transform dh_to_transform(const DH& dh, const float& theta);
}