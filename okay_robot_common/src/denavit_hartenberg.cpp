#include "okay_robot_common/denavit_hartenberg.hpp"
#include <Eigen/Dense>

namespace OkayRobot {
float DHChain::a(const int& index) const { return this->dh_chain_[index].a; }
float DHChain::d(const int& index) const { return this->dh_chain_[index].d; }
float DHChain::alpha(const int& index) const { return this->dh_chain_[index].alpha; }
float DHChain::theta(const int& index) const { return this->dh_chain_[index].theta; }
const DH DHChain::dh(const int& index) const { return this->dh_chain_[index]; };
int DHChain::length() const { return this->dh_chain_.size(); }

Transform dh_to_transform(const DH& dh, const float& theta)
{
    const float new_theta = dh.theta + theta;

    const float sin_theta = std::sin(new_theta);
    const float cos_theta = std::cos(new_theta);
    const float sin_alpha = std::sin(dh.alpha);
    const float cos_alpha = std::cos(dh.alpha);

    /** TODO: find a cleaner way to format this? */
    Eigen::Matrix4f m;
    m << cos_theta, -sin_theta * cos_alpha, sin_theta * sin_alpha, cos_theta * dh.a, sin_theta,
        cos_theta * cos_alpha, -cos_theta * sin_alpha, sin_theta * dh.a, 0.0, sin_alpha, cos_alpha,
        dh.d, 0.0, 0.0, 0.0, 1.0;

    return Transform(m);
}
}