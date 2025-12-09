#include "okay_robot_control/control/direct_controller.hpp"

DirectController::DirectController()
{
    this->current_goal_ = std::make_unique<OkayRobot::JointPose>(
        std::vector<float>({ M_PI_2, M_PI_2, 3.0 * M_PI_2, M_PI, M_PI_2, M_PI }));
}

void DirectController::set_goal_state(const OkayRobot::JointPose& goal_state)
{
    this->current_goal_ = std::make_unique<OkayRobot::JointPose>(goal_state);
}

OkayRobot::JointPose DirectController::step_control_loop(
    const OkayRobot::Observation& current_state)
{
    return OkayRobot::JointPose(this->current_goal_->joint_positions);
}