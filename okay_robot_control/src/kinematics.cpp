#include "okay_robot_control/kinematics.hpp"
#include "okay_robot_common/denavit_hartenberg.hpp"

OkayRobot::Transform Kinematics::get_forward(const OkayRobot::Pose& robot_pose)
{
    OkayRobot::Transform running_tf = OkayRobot::Transform();
    OkayRobot::Transform dh_tf;
    for (int i = 0; i < this->dh_chain_.size(); i++) {
        dh_tf = dh_to_transform(this->dh_chain_[i], robot_pose.joint_positions[i]);
        running_tf = running_tf.forward(dh_tf);
    }

    return running_tf;
}