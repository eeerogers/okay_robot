#pragma once

#include <memory>

#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/observation.hpp"
#include "okay_robot_common/pose.hpp"
#include "okay_robot_control/control/controller.hpp"

namespace OkayRobot {
class DirectController : public Controller {
public:
    DirectController();

    virtual void setGoalState(const JointPose& goal_state) override;
    virtual JointPose stepControlLoop(const Observation& current_state) override;

private:
    std::unique_ptr<JointPose> current_goal_;
};
}