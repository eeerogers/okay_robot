#include "okay_robot_common/denavit_hartenberg.hpp"
#include <Eigen/Dense>

Transform DH::to_transform(const double& theta)
{
    const double new_theta = this->theta_ + theta;

    const double sin_theta = sin(new_theta);
    const double cos_theta = cos(new_theta);
    const double sin_alpha = sin(this->alpha_);
    const double cos_alpha = cos(this->alpha_);

    /** TODO: find a cleaner way to format this? */
    Eigen::Matrix4d m;
    m << cos_theta, -sin_theta, 0.0, this->a_, sin_theta * cos_alpha, cos_theta * cos_alpha,
        -sin_alpha, -sin_alpha * this->d_, sin_theta * sin_alpha, cos_theta * sin_alpha, cos_alpha,
        cos_alpha * this->d_, 0.0, 0.0, 0.0, 1.0;

    return Transform(m);
}