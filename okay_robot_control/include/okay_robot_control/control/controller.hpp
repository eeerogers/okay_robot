#pragma once

#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/observation.hpp"

// protocol
class Controller {
public:
    virtual void set_goal_state(const OkayRobot::JointPose& goal_state) = 0;
    virtual OkayRobot::JointPose step_control_loop(const OkayRobot::Observation& current_state) = 0;
};