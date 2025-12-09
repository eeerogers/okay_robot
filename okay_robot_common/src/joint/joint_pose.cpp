#include <algorithm>
#include <cmath>

#include "okay_robot_common/joint/joint_pose.hpp"

namespace OkayRobot {
bool pose_is_valid(const JointPose pose)
{
    return !std::any_of(pose.joint_positions.begin(), pose.joint_positions.end(),
        [](float x) { return std::isnan(x); });
}
}