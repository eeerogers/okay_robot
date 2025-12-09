#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class Twist {
public:
    Twist(Eigen::Vector3f linear, Eigen::Vector3f angular)
        : linear(linear)
        , angular(angular) { };

    const Eigen::Vector3f linear;
    const Eigen::Vector3f angular;
};
}