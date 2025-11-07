#include <algorithm>
#include <cmath>

#include "okay_robot_common/robot_state.hpp"

namespace OkayRobot {
bool pose_is_valid(const Pose pose)
{
    return !std::any_of(pose.joint_positions.begin(), pose.joint_positions.end(),
        [](float x) { return std::isnan(x); });
}
}