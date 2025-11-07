#pragma once

#include <vector>

#include "okay_robot_common/description.hpp"
#include "okay_robot_common/robot_state.hpp"
#include "okay_robot_common/transform.hpp"

class Kinematics {
public:
    Kinematics()
        : description_(this->setup_robot_description_()) { };

    OkayRobot::Pose get_inverse(const OkayRobot::Transform& eef_transform);
    OkayRobot::Transform get_forward(const OkayRobot::Pose& robot_pose);

private:
    const OkayRobot::Description description_;

    const OkayRobot::Description setup_robot_description_();
};