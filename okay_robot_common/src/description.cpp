#include "okay_robot_common/description.hpp"

namespace OkayRobot {

float Description::upper_bound(const int& index) const { return this->joints_[index].bounds.upper; }

float Description::lower_bound(const int& index) const { return this->joints_[index].bounds.lower; }

float Description::dh_a(const int& index) const { return this->joints_[index].dh.a; }

float Description::dh_d(const int& index) const { return this->joints_[index].dh.d; }

float Description::dh_alpha(const int& index) const { return this->joints_[index].dh.alpha; }

float Description::dh_theta(const int& index) const { return this->joints_[index].dh.theta; }

const DenavitHartenberg Description::dh(const int& index) const { return this->joints_[index].dh; }

int Description::length() const { return this->joints_.size(); }

bool Description::pose_is_valid(const OkayRobot::Pose pose) const
{
    for (int i = 0; i < pose.joint_positions.size(); i++) {
        if (std::isnan(pose.joint_positions[i]))
            return false;

        if (pose.joint_positions[i] > this->upper_bound(i)
            || pose.joint_positions[i] < this->lower_bound(i))
            return false;
    }

    return true;
}
}