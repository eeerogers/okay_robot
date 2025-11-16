#include <algorithm>
#include <cmath>

#include "okay_robot_common/robot_state.hpp"

namespace OkayRobot {

Eigen::Vector<float, 6> pose_to_eigen_vector(const Pose pose)
{
    Eigen::Vector<float, 6> as_eigen;
    std::copy(pose.joint_positions.begin(), pose.joint_positions.end(), as_eigen.data());

    return as_eigen;
}

bool pose_is_valid(const Pose pose)
{
    return !std::any_of(pose.joint_positions.begin(), pose.joint_positions.end(),
        [](float x) { return std::isnan(x); });
}
}