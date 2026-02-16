#pragma once

#include <Eigen/Dense>
#include <vector>

#include "okay_robot_common/description.hpp"
#include "okay_robot_common/joint/joint_pose.hpp"
#include "okay_robot_common/transform/transform.hpp"

namespace OkayRobot {
class Kinematics {
public:
    Kinematics()
        : description_(this->setupRobotDescription_())
        , joint_offsets_(this->setupJointOffsets_()) { };

    const JointPose getInverse(const Transform& eef_transform, const JointPose& last_pose) const;
    const Transform getForward(const JointPose& robot_pose) const;
    Eigen::Matrix<float, 6, 6> getJacobian(const JointPose& robot_pose) const;

    bool poseIsValid(const JointPose pose) const;

private:
    const Description description_;
    const Eigen::Vector<float, 6> joint_offsets_;

    const Description setupRobotDescription_();
    const Eigen::Vector<float, 6> setupJointOffsets_();
};
}