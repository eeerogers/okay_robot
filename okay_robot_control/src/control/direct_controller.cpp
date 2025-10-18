#include "okay_robot_control/control/direct_controller.hpp"

DirectController::DirectController()
{
    this->current_goal_ = std::make_unique<OkayRobotGoal>(std::vector<float>(7, 0.0));
}

void DirectController::set_goal_state(const OkayRobotGoal& goal_state)
{
    this->current_goal_ = std::make_unique<OkayRobotGoal>(goal_state);
}

OkayRobotCommand DirectController::step_control_loop(const OkayRobotObservation& current_state)
{
    return OkayRobotCommand(this->current_goal_->joint_positions);
}