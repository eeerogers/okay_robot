#pragma once

#include "okay_robot_common/pose.hpp"

namespace OkayRobot {
// protocol
class PathPlanner {
    virtual void setGoalState(Pose& goal_state) = 0;
};
}