#pragma once

#include <vector>

#include "okay_robot_common/denavit_hartenberg.hpp"
#include "okay_robot_common/okay_robot_state.hpp"
#include "okay_robot_common/transform.hpp"

class Kinematics {
public:
    Kinematics()
        : dh_chain_(this->setup_dh_chain_()) { };

    OkayRobot::Pose get_inverse(const OkayRobot::Transform& eef_transform);
    OkayRobot::Transform get_forward(const OkayRobot::Pose& robot_pose);

private:
    const OkayRobot::DHChain dh_chain_;

    const OkayRobot::DHChain setup_dh_chain_();
};