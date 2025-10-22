#include <cmath>
#include <memory>

#include "okay_robot_control/kinematics.hpp"

// TODO: un-hardcode this
Kinematics::Kinematics()
{
    float d0 = 0.041;

    float d1 = 0.042;
    float a1 = 0.040;
    float alpha1 = M_PI / 2.0;
    float offset1 = M_PI / 2.0;

    float a2_a = 0.150;
    float a2_b = 0.028;
    float phi2 = std::tan(a2_b / a2_a);
    float a2 = std::sqrt(a2_a * a2_a + a2_b * a2_b);
    float theta2 = (M_PI / 2.0) + phi2;
    float offset2 = M_PI / 2.0;

    float a3 = 0.060;
    float alpha3 = M_PI / 2.0;
    float theta3 = -phi2;
    float offset3 = 3.0 * M_PI / 2.0;

    float d4 = 0.155;
    float alpha4 = -M_PI / 2.0;
    float offset4 = M_PI;

    float alpha5 = M_PI / 2.0;
    float offset5 = M_PI / 2.0;

    float d6 = 0.065;
    float offset6 = M_PI;

    float d7 = 0.079;

    //                      a,  d,  alpha,  theta
    auto j0 = OkayRobot::DH(0.0, d0, 0.0, 0.0);
    auto j1 = OkayRobot::DH(a1, d1, alpha1, -offset1);
    auto j2 = OkayRobot::DH(a2, 0.0, 0.0, theta2 - offset2);
    auto j3 = OkayRobot::DH(a3, 0.0, alpha3, theta3 - offset3);
    auto j4 = OkayRobot::DH(0.0, d4, alpha4, -offset4);
    auto j5 = OkayRobot::DH(0.0, 0.0, alpha5, -offset5);
    auto j6 = OkayRobot::DH(0.0, d6, 0.0, -offset6);
    auto j7 = OkayRobot::DH(0.0, d7, 0.0, 0.0);

    this->dh_chain_ = std::vector<OkayRobot::DH>({ j0, j1, j2, j3, j4, j5, j6, j7 });
}

// TODO: implement
// OkayRobot::Pose Kinematics::get_inverse(const OkayRobot::Transform& eef_transform)
// {
//     //
// }

OkayRobot::Transform Kinematics::get_forward(const OkayRobot::Pose& robot_pose)
{
    // start with the transform from world to joint1
    auto running_tf
        = std::make_unique<OkayRobot::Transform>(dh_to_transform(this->dh_chain_[0], 0.0));

    // iterate through each joint in the chain (6)
    // TODO: don't hardcode the number of joints?
    for (int i = 0; i < 6; i++) {
        auto dh_tf = dh_to_transform(this->dh_chain_[i + 1], robot_pose.joint_positions[i]);
        running_tf = std::make_unique<OkayRobot::Transform>(running_tf->forward(dh_tf));
    }

    // the transforms after kinematic chain just gets added to the end
    OkayRobot::Transform eef_dh_tf = dh_to_transform(this->dh_chain_.back(), 0.0);
    return running_tf->forward(eef_dh_tf);
}