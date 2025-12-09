#pragma once

#include "okay_robot_common/pose.hpp"

// protocol
class PathPlanner {
    virtual void set_goal_state(OkayRobot::Pose& goal_state) = 0;
};