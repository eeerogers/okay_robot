#include <cmath>
#include <iostream>
#include <memory>

#include "okay_robot_control/kinematics.hpp"

namespace OkayRobot {

const JointPose Kinematics::getInverse(
    const Transform& eef_transform, const JointPose& last_pose) const
{
    Eigen::Vector3f eef_offset
        = (this->description_.dhD(6) + this->description_.dhD(7)) * eef_transform.rotation().z();
    const Eigen::Vector3f offset_position = eef_transform.position().vector() - eef_offset;
    const Eigen::Matrix3f rotation = eef_transform.rotation().matrix();

    float x = offset_position[0];
    float y = offset_position[1];
    float z = offset_position[2];

    float s = z - (this->description_.dhD(0) + this->description_.dhD(1));
    float r = std::sqrt(x * x + y * y) - this->description_.dhA(1);
    float phi2 = std::atan(0.028 / 0.150);
    float a2 = this->description_.dhA(2);
    float a3 = this->description_.dhA(3);
    float d4 = this->description_.dhD(4);

    // j1
    float theta1 = std::atan2(y, x);

    // j2
    float alpha = std::atan(s / r);
    float d_beta
        = (a2 * a2 + (r * r + s * s) - (a3 * a3 + d4 * d4)) / (2.0 * a2 * std::sqrt(r * r + s * s));
    float beta = std::atan2(std::sqrt(1.0 - d_beta * d_beta), d_beta);
    float theta2 = -M_PI_2 - phi2 + (alpha + beta);

    // j3
    float d_theta3 = (a2 * a2 + (a3 * a3 + d4 * d4) - (s * s + r * r))
        / (2.0 * a2 * std::sqrt(a3 * a3 + d4 * d4));
    float theta3 = -M_PI_2 - phi2 + std::atan2(std::sqrt(1.0 - d_theta3 * d_theta3), d_theta3);

    // j4
    float r13 = -rotation(0, 2) * std::sin(theta2 + theta3) * std::cos(theta1)
        - rotation(1, 2) * std::sin(theta1) * std::sin(theta2 + theta3)
        + rotation(2, 2) * std::cos(theta2 + theta3);
    float r23 = rotation(0, 2) * std::sin(theta1) - rotation(1, 2) * std::cos(theta1);
    float theta4_pos = std::atan2(r23, r13);
    float theta4_neg = std::atan2(-r23, -r13);

    // j5
    float r33 = rotation(0, 2) * (std::cos(theta2 + theta3) * std::cos(theta1))
        + rotation(1, 2) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 2) * (std::sin(theta2 + theta3));
    float theta5_pos = std::atan2(std::sqrt(1.0 - r33 * r33), r33);
    float theta5_neg = std::atan2(-std::sqrt(1.0 - r33 * r33), r33);

    // j6
    float r31 = rotation(0, 0) * (std::cos(theta1) * std::cos(theta2 + theta3))
        + rotation(1, 0) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 0) * (std::sin(theta2 + theta3));
    float r32 = rotation(0, 1) * (std::cos(theta1) * std::cos(theta2 + theta3))
        + rotation(1, 1) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 1) * (std::sin(theta2 + theta3));
    float theta6_pos = std::atan2(r32, -r31);
    float theta6_neg = std::atan2(-r32, r31);

    // determine which configuration to use
    Eigen::Vector<float, 6> pose_pos;
    pose_pos << theta1, theta2, theta3, theta4_pos, theta5_pos, theta6_pos;
    pose_pos += this->joint_offsets_;

    Eigen::Vector<float, 6> pose_neg;
    pose_neg << theta1, theta2, theta3, theta4_neg, theta5_neg, theta6_neg;
    pose_neg += this->joint_offsets_;

    if ((last_pose.joint_positions - pose_pos).lpNorm<1>()
        <= (last_pose.joint_positions - pose_neg).lpNorm<1>())
        return JointPose(pose_pos);
    else
        return JointPose(pose_neg);
}

const Transform Kinematics::getForward(const JointPose& robot_pose) const
{
    // start with the transform from world to joint1
    Transform running_tf(this->description_.dh(0), 0.0);

    // iterate through each joint in the chain (6)
    // TODO: don't hardcode the number of joints?
    for (int i = 0; i < 6; i++) {
        auto dh_tf = Transform(this->description_.dh(i + 1), robot_pose.joint_positions[i]);
        running_tf = running_tf.forward(dh_tf);
    }

    // the transforms after kinematic chain just gets added to the end
    auto eef_dh_tf = Transform(this->description_.dh(7), 0.0);
    return running_tf.forward(eef_dh_tf);
}

Eigen::Matrix<float, 6, 6> Kinematics::getJacobian(const JointPose& robot_pose) const
{
    // J = [Jv]
    //     [Jw]

    // Jv = [z_i-1 x (o_n - o_i-1)]
    // Jw = [z_i-1]

    const Transform tf_eef = this->getForward(robot_pose);

    Eigen::Matrix<float, 6, 6> jacobian = Eigen::Matrix<float, 6, 6>::Zero(6, 6);
    Transform dh_tf(this->description_.dh(0), 0.0);
    Transform running_tf(dh_tf);

    for (int i = 0; i < 6; i++) {
        // populate Jv and Jw in the column of the jacobian
        jacobian.block<3, 1>(0, i) = running_tf.rotation().z().cross(
            tf_eef.position().vector() - running_tf.position().vector());
        jacobian.block<3, 1>(3, i) = running_tf.rotation().z();

        dh_tf = Transform(this->description_.dh(i + 1), robot_pose.joint_positions[i]);
        running_tf = running_tf.forward(dh_tf);
    }

    return jacobian;
}

bool Kinematics::poseIsValid(const JointPose pose) const
{
    for (int i = 0; i < pose.joint_positions.size(); i++) {
        if (std::isnan(pose.joint_positions[i])) {
            return false;
        }

        // TODO: make this work
        // if (pose.joint_positions[i] > this->description_.upperBound(i + 1)
        //     || pose.joint_positions[i] < this->description_.lowerBound(i + 1)) {
        //     return false;
        // }
    }

    return true;
}

const Description Kinematics::setupRobotDescription_()
{
    // d0 = 0.039;
    DenavitHartenberg j0_dh { 0.0, 0.039, 0.0, 0.0 };
    Bounds j0_bounds { 0.0, 0.0 };
    JointDescription joint0 { j0_dh, j0_bounds };

    // a1 = 0.040;
    // d1 = 0.042;
    // alpha1 = 90deg;
    // offset1 = 90deg;
    DenavitHartenberg j1_dh { 0.040, 0.042, M_PI_2, -M_PI_2 };
    Bounds j1_bounds { -M_PI_2, M_PI_2 };
    JointDescription joint1 { j1_dh, j1_bounds };

    // a2_a = 0.150;
    // a2_b = 0.028;
    // a2 = std::sqrt(a2_a * a2_a + a2_b * a2_b);
    // phi2 = std::tan(a2_b / a2_a);
    // theta2 = 90deg + phi2;
    // offset2 = 90deg;
    DenavitHartenberg j2_dh { std::sqrt(0.15 * 0.15 + 0.028 * 0.028), 0.0, 0.0,
        std::tan(0.028 / 0.15) };
    Bounds j2_bounds { -M_PI_2, M_PI_2 };
    JointDescription joint2 { j2_dh, j2_bounds };

    // a3 = 0.060;
    // alpha3 = 90deg;
    // theta3 = -phi2;
    // offset3 = 270deg;
    DenavitHartenberg j3_dh { 0.06, 0.0, M_PI_2, -std::tan(0.028 / 0.15) - (3.0 * M_PI_2) };
    Bounds j3_bounds { -M_PI_2, M_PI_2 };
    JointDescription joint3 { j3_dh, j3_bounds };

    // d4 = 0.155;
    // alpha4 = -90deg;
    // offset4 = 180deg;
    DenavitHartenberg j4_dh { 0.0, 0.155, -M_PI_2, -M_PI };
    Bounds j4_bounds { -M_PI, M_PI };
    JointDescription joint4 { j4_dh, j4_bounds };

    // alpha5 = 90deg;
    // offset5 = 90deg;
    DenavitHartenberg j5_dh { 0.0, 0.0, M_PI_2, -M_PI_2 };
    Bounds j5_bounds { -M_PI_2, M_PI_2 };
    JointDescription joint5 { j5_dh, j5_bounds };

    // d6 = 0.065;
    // offset6 = 180deg;
    DenavitHartenberg j6_dh { 0.0, 0.065, 0.0, -M_PI };
    Bounds j6_bounds { -M_PI, M_PI };
    JointDescription joint6 { j6_dh, j6_bounds };

    // d7 = 0.079;
    DenavitHartenberg j7_dh { 0.0, 0.079, 0.0, 0.0 };
    Bounds j7_bounds { 0.0, 0.0 };
    JointDescription joint7 { j7_dh, j7_bounds };

    auto joint_chain = std::vector<JointDescription> { joint0, joint1, joint2, joint3, joint4,
        joint5, joint6, joint7 };
    return Description(joint_chain);
}

const Eigen::Vector<float, 6> Kinematics::setupJointOffsets_()
{
    // offsets to match the real robot
    // theta1 = 90.0
    // theta2 = 90.0
    // theta3 = 270.0
    // theta4 = 180.0
    // theta5 = 90.0
    // theta6 = 180.0

    Eigen::Vector<float, 6> joint_offsets;
    joint_offsets << M_PI_2, M_PI_2, 3.0 * M_PI_2, M_PI, M_PI_2, M_PI;
    return joint_offsets;
}

}