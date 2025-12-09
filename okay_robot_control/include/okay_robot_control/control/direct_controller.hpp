#pragma once

#include <memory>

#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/observation.hpp"
#include "okay_robot_common/pose.hpp"
#include "okay_robot_control/control/controller.hpp"

class DirectController : public Controller {
public:
    DirectController();

    virtual void set_goal_state(const OkayRobot::JointPose& goal_state) override;
    virtual OkayRobot::JointPose step_control_loop(
        const OkayRobot::Observation& current_state) override;

private:
    std::unique_ptr<OkayRobot::JointPose> current_goal_;
};