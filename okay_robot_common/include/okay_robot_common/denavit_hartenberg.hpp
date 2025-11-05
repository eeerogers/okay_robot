#pragma once

#include <Eigen/Dense>
#include <memory>

#include "okay_robot_common/transform.hpp"

namespace OkayRobot {
class DH {
public:
    DH(const float& a, const float& d, const float& alpha, const float& theta)
        : a(a)
        , d(d)
        , alpha(alpha)
        , theta(theta) { };

    const float a;
    const float d;
    const float alpha;
    const float theta;
};

class DHChain {
public:
    DHChain(const std::vector<DH>& dh_chain)
        : dh_chain_(dh_chain) { };

    float a(const int& index) const;
    float d(const int& index) const;
    float alpha(const int& index) const;
    float theta(const int& index) const;
    const DH dh(const int& index) const;

    int length() const;

private:
    const std::vector<DH> dh_chain_;
};

OkayRobot::Transform dh_to_transform(const DH& dh, const float& theta);
}