#pragma once

#include "okay_robot_common/okay_robot_state.hpp"

// protocol
class PathPlanner {
    virtual void set_goal_state(OkayRobotGoal& goal_state) = 0;
};