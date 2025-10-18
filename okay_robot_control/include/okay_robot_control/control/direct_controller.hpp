#pragma once

#include <memory>

#include "okay_robot_common/okay_robot_state.hpp"
#include "okay_robot_control/control/controller.hpp"

class DirectController : public Controller {
public:
    DirectController();

    virtual void set_goal_state(const OkayRobotGoal& goal_state) override;
    virtual OkayRobotCommand step_control_loop(const OkayRobotObservation& current_state) override;

private:
    std::unique_ptr<OkayRobotGoal> current_goal_;
};