#pragma once

#include <Eigen/Dense>
#include <vector>

namespace OkayRobot {
class JointVelocities {
public:
    JointVelocities(const Eigen::Vector<float, 6> vel)
        : joint_velocities(vel) { };

    const Eigen::Vector<float, 6> joint_velocities;
};
}