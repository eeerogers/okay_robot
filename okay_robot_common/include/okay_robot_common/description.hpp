#pragma once

#include <memory>
#include <vector>

#include "okay_robot_common/pose.hpp"
#include "okay_robot_common/transform/denavit_hartenberg.hpp"

namespace OkayRobot {

struct Bounds {
    const float lower;
    const float upper;
};

struct JointDescription {
    const DenavitHartenberg dh;
    const Bounds bounds;
};

class Description {
public:
    Description(const std::vector<JointDescription> joints)
        : joints_(joints) { };

    inline float upperBound(const int& index) const { return this->joints_[index].bounds.upper; };
    inline float lowerBound(const int& index) const { return this->joints_[index].bounds.lower; };

    inline float dhA(const int& index) const { return this->joints_[index].dh.a; };
    inline float dhD(const int& index) const { return this->joints_[index].dh.d; };
    inline float dhAlpha(const int& index) const { return this->joints_[index].dh.alpha; };
    inline float dhTheta(const int& index) const { return this->joints_[index].dh.theta; };
    inline const DenavitHartenberg dh(const int& index) const { return this->joints_[index].dh; };

    inline int length() const { return this->joints_.size(); };

private:
    const std::vector<JointDescription> joints_;
};
}