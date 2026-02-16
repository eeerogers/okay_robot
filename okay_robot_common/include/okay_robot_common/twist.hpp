#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
struct Twist {
    const Eigen::Vector3f linear;
    const Eigen::Vector3f angular;
};
}