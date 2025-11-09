#include "okay_robot_control/control/direct_controller.hpp"

DirectController::DirectController()
{
    this->current_goal_ = std::make_unique<OkayRobot::Pose>(
        std::vector<float>({ 1.57, 1.57, 4.71, 3.14, 1.57, 3.14, 0.15 }));
}

void DirectController::set_goal_state(const OkayRobot::Pose& goal_state)
{
    this->current_goal_ = std::make_unique<OkayRobot::Pose>(goal_state);
}

OkayRobot::Command DirectController::step_control_loop(const OkayRobot::Observation& current_state)
{
    return OkayRobot::Command(this->current_goal_->joint_positions);
}