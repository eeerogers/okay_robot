#pragma once

#include <Eigen/Dense>

#include "okay_robot_common/transform.hpp"

namespace OkayRobot {
class DH {
public:
    DH(const float a, const float d, const float alpha, const float theta)
        : a(a)
        , d(d)
        , alpha(alpha)
        , theta(theta) { };

    const float a;
    const float d;
    const float alpha;
    const float theta;
};

OkayRobot::Transform dh_to_transform(const DH& dh, const float& theta);
}