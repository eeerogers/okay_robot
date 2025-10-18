#pragma once

#include "okay_robot_common/okay_robot_state.hpp"

// protocol
class Controller {
public:
    virtual void set_goal_state(const OkayRobotGoal& goal_state) = 0;
    virtual OkayRobotCommand step_control_loop(const OkayRobotObservation& current_state) = 0;
};