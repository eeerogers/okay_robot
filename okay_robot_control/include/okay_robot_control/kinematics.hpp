#pragma once

#include <Eigen/Dense>
#include <vector>

#include "okay_robot_common/description.hpp"
#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/transform/transform.hpp"

class Kinematics {
public:
    Kinematics()
        : description_(this->setup_robot_description_())
        , joint_offsets_(this->setup_joint_offsets_()) { };

    OkayRobot::JointPose get_inverse(
        const OkayRobot::Transform& eef_transform, const OkayRobot::JointPose& last_pose);
    OkayRobot::Transform get_forward(const OkayRobot::JointPose& robot_pose);

private:
    const OkayRobot::Description description_;
    const Eigen::Vector<float, 6> joint_offsets_;

    const OkayRobot::Description setup_robot_description_();
    const Eigen::Vector<float, 6> setup_joint_offsets_();
};