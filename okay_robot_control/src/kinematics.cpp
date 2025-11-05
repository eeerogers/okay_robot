#include <cmath>
#include <memory>

#include "okay_robot_control/kinematics.hpp"

OkayRobot::Pose Kinematics::get_inverse(const OkayRobot::Transform& eef_transform)
{
    Eigen::Vector3f z_dir = eef_transform.rotation().block<3, 1>(0, 2);
    Eigen::Vector3f eef_offset = (this->dh_chain_.d(6) + this->dh_chain_.d(7)) * z_dir;
    Eigen::Vector3f offset_position = eef_transform.position() - eef_offset;
    Eigen::Matrix3f rotation = eef_transform.rotation();

    float x = offset_position[0];
    float y = offset_position[1];
    float z = offset_position[2];

    float s = z - (this->dh_chain_.d(0) + this->dh_chain_.d(1));
    float r = std::sqrt(x * x + y * y) - this->dh_chain_.a(1);
    float phi2 = std::atan(0.028 / 0.150);
    float a2 = this->dh_chain_.a(2);
    float a3 = this->dh_chain_.a(3);
    float d4 = this->dh_chain_.d(4);

    // j1
    float theta1 = std::atan2(y, x);

    // j2
    float alpha = std::atan(s / r);
    float d_beta
        = (a2 * a2 + (r * r + s * s) - (a3 * a3 + d4 * d4)) / (2.0 * a2 * std::sqrt(r * r + s * s));
    float beta = std::atan2(std::sqrt(1.0 - d_beta * d_beta), d_beta);
    float theta2 = -(M_PI / 2.0) - phi2 + (alpha + beta);

    // j3
    float d_theta3 = (a2 * a2 + (a3 * a3 + d4 * d4) - (s * s + r * r))
        / (2.0 * a2 * std::sqrt(a3 * a3 + d4 * d4));
    float theta3 = -(M_PI / 2) - phi2 + std::atan2(std::sqrt(1.0 - d_theta3 * d_theta3), d_theta3);

    // j4
    float r13 = -rotation(0, 2) * std::sin(theta2 + theta3) * std::cos(theta1)
        - rotation(1, 2) * std::sin(theta1) * std::sin(theta2 + theta3)
        + rotation(2, 2) * std::cos(theta2 + theta3);
    float r23 = rotation(0, 2) * std::sin(theta1) - rotation(1, 2) * std::cos(theta1);
    float theta4 = std::atan2(r23, r13);

    // j5
    float r33 = rotation(0, 2) * (std::cos(theta2 + theta3) * std::cos(theta1))
        + rotation(1, 2) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 2) * (std::sin(theta2 + theta3));
    float theta5 = std::atan2(std::sqrt(1.0 - r33 * r33), r33);

    // j6
    float r31 = rotation(0, 0) * (std::cos(theta1) * std::cos(theta2 + theta3))
        + rotation(1, 0) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 0) * (std::sin(theta2 + theta3));
    float r32 = rotation(0, 1) * (std::cos(theta1) * std::cos(theta2 + theta3))
        + rotation(1, 1) * (std::sin(theta1) * std::cos(theta2 + theta3))
        + rotation(2, 1) * (std::sin(theta2 + theta3));
    float theta6 = std::atan2(r32, -r31);

    // offsets to match the real robot
    // theta1 = 90.0
    // theta2 = 90.0
    // theta3 = 270.0
    // theta4 = 180.0
    // theta5 = 90.0
    // theta6 = 180.0

    auto joint_positions = std::vector<float>(
        { theta1 + float(M_PI / 2.0), theta2 + float(M_PI / 2.0), theta3 + float(3.0 * M_PI / 2.0),
            theta4 + float(M_PI), theta5 + float(M_PI / 2.0), theta6 + float(M_PI) });
    return OkayRobot::Pose(joint_positions);
}

OkayRobot::Transform Kinematics::get_forward(const OkayRobot::Pose& robot_pose)
{
    // start with the transform from world to joint1
    auto running_tf
        = std::make_unique<OkayRobot::Transform>(dh_to_transform(this->dh_chain_.dh(0), 0.0));

    // iterate through each joint in the chain (6)
    // TODO: don't hardcode the number of joints?
    for (int i = 0; i < 6; i++) {
        auto dh_tf = dh_to_transform(this->dh_chain_.dh(i + 1), robot_pose.joint_positions[i]);
        running_tf = std::make_unique<OkayRobot::Transform>(running_tf->forward(dh_tf));
    }

    // the transforms after kinematic chain just gets added to the end
    OkayRobot::Transform eef_dh_tf = dh_to_transform(this->dh_chain_.dh(7), 0.0);
    return running_tf->forward(eef_dh_tf);
}

const OkayRobot::DHChain Kinematics::setup_dh_chain_()
{
    // d0 = 0.039;
    auto j0 = OkayRobot::DH(0.0, 0.039, 0.0, 0.0);

    // a1 = 0.040;
    // d1 = 0.042;
    // alpha1 = 90deg;
    // offset1 = 90deg;
    auto j1 = OkayRobot::DH(0.040, 0.042, M_PI / 2.0, -M_PI / 2.0);

    // a2_a = 0.150;
    // a2_b = 0.028;
    // a2 = std::sqrt(a2_a * a2_a + a2_b * a2_b);
    // phi2 = std::tan(a2_b / a2_a);
    // theta2 = 90deg + phi2;
    // offset2 = 90deg;
    auto j2
        = OkayRobot::DH(std::sqrt(0.15 * 0.15 + 0.028 * 0.028), 0.0, 0.0, std::tan(0.028 / 0.15));

    // a3 = 0.060;
    // alpha3 = 90deg;
    // theta3 = -phi2;
    // offset3 = 270deg;
    auto j3 = OkayRobot::DH(0.06, 0.0, M_PI / 2.0, -std::tan(0.028 / 0.15) - (3.0 * M_PI / 2.0));

    // d4 = 0.155;
    // alpha4 = -90deg;
    // offset4 = 90deg;
    auto j4 = OkayRobot::DH(0.0, 0.155, -M_PI / 2.0, -M_PI);

    // alpha5 = 90deg;
    // offset5 = 90deg;
    auto j5 = OkayRobot::DH(0.0, 0.0, M_PI / 2.0, -M_PI / 2.0);

    // d6 = 0.065;
    // offset6 = 90deg;
    auto j6 = OkayRobot::DH(0.0, 0.065, 0.0, -M_PI);

    // d7 = 0.079;
    auto j7 = OkayRobot::DH(0.0, 0.079, 0.0, 0.0);

    auto dh_chain = std::vector<OkayRobot::DH>({ j0, j1, j2, j3, j4, j5, j6, j7 });
    return OkayRobot::DHChain(dh_chain);
}