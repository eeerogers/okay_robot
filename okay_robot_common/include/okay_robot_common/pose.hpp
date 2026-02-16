#pragma once

#include <Eigen/Dense>
#include <vector>

namespace OkayRobot {
class Pose {
public:
    Pose(const std::vector<float> pos)
        : joint_positions(pos) { };
    Pose(const Eigen::Vector<float, 6> pos)
        : joint_positions(std::vector<float>(pos.data(), pos.data() + pos.size())) { };

    const std::vector<float> joint_positions;
};
}