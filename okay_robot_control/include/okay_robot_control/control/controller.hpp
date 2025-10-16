#pragma once

#include "okay_robot_common/okay_robot_state.hpp"

// protocol
class Controller {
    virtual void set_goal_state(OkayRobotGoal& goal_state) = 0;
    virtual void step_control_loop(OkayRobotObservation& current_state) = 0;
};