#pragma once

#include "okay_robot_common/command.hpp"
#include "okay_robot_common/observation.hpp"
#include "okay_robot_common/pose.hpp"

// protocol
class Controller {
public:
    virtual void set_goal_state(const OkayRobot::Pose& goal_state) = 0;
    virtual OkayRobot::JointPositionCommand step_control_loop(
        const OkayRobot::Observation& current_state)
        = 0;
};