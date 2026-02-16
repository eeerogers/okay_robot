#pragma once

#include <Eigen/Dense>
#include <vector>

namespace OkayRobot {
class JointPose {
public:
    JointPose(const Eigen::Vector<float, 6> pos)
        : joint_positions(pos) { };
    JointPose(const std::vector<float> pos)
        : joint_positions(pos.data()) { };

    const Eigen::Vector<float, 6> joint_positions;
};
}