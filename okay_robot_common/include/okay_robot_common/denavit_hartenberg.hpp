#pragma once

#include "okay_robot_common/transform.hpp"
#include <Eigen/Dense>

class DH {
public:
    DH(const double& a, const double& d, const double& alpha, const double& theta)
        : a_(a)
        , d_(d)
        , alpha_(alpha)
        , theta_(theta) { };

    Transform to_transform(const double& theta);

private:
    const double a_;
    const double d_;
    const double alpha_;
    const double theta_;
};