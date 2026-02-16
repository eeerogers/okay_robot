#include "okay_robot_control/control/direct_controller.hpp"

namespace OkayRobot {

DirectController::DirectController()
{
    this->current_goal_ = std::make_unique<JointPose>(
        std::vector<float>({ M_PI_2, M_PI_2, 3.0 * M_PI_2, M_PI, M_PI_2, M_PI }));
}

void DirectController::setGoalState(const JointPose& goal_state)
{
    this->current_goal_ = std::make_unique<JointPose>(goal_state);
}

JointPose DirectController::stepControlLoop(const Observation& current_state)
{
    return JointPose(this->current_goal_->joint_positions);
}

}