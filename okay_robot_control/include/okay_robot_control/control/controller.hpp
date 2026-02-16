#pragma once

#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/observation.hpp"

namespace OkayRobot {
// protocol
class Controller {
public:
    virtual void setGoalState(const JointPose& goal_state) = 0;
    virtual JointPose stepControlLoop(const Observation& current_state) = 0;
};
}