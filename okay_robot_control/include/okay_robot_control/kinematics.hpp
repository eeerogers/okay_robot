#pragma once

#include <vector>

#include "okay_robot_common/denavit_hartenberg.hpp"
#include "okay_robot_common/okay_robot_state.hpp"
#include "okay_robot_common/transform.hpp"

class Kinematics {
public:
    void get_inverse(const OkayRobot::Transform& eef_transform);
    OkayRobot::Transform get_forward(const OkayRobot::Pose& robot_pose);

private:
    const std::vector<OkayRobot::DH> dh_chain_;
};