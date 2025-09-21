#pragma once

#include "okay_robot_control/okay_robot_state.hpp"

// protocol
class ControlLoop {
public:
    virtual void set_goal_state(OkayRobotGoal goal_state) = 0;
    virtual void step_control_loop(OkayRobotState current_state) = 0;
};