#pragma once

#include "okay_robot_common/okay_robot_state.hpp"
#include "okay_robot_control/control/controller.hpp"

class DirectController : Controller {
    virtual void set_goal_state(OkayRobotGoal& goal_state) override;
    virtual void step_control_loop(OkayRobotObservation& current_state) override;
};