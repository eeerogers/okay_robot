#pragma once

#include <memory>

#include "okay_robot_common/robot_state.hpp"
#include "okay_robot_control/control/controller.hpp"

class DirectController : public Controller {
public:
    DirectController();

    virtual void set_goal_state(const OkayRobot::Pose& goal_state) override;
    virtual OkayRobot::Command step_control_loop(
        const OkayRobot::Observation& current_state) override;

private:
    std::unique_ptr<OkayRobot::Pose> current_goal_;
};