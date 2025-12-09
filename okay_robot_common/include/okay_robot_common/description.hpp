#pragma once

#include <memory>
#include <vector>

#include "okay_robot_common/pose.hpp"
#include "okay_robot_common/transform/denavit_hartenberg.hpp"

namespace OkayRobot {

struct Bounds {
    const float upper;
    const float lower;
};

class JointDescription {
public:
    JointDescription(const OkayRobot::DenavitHartenberg dh, const Bounds bounds)
        : dh(dh)
        , bounds(bounds) { };

    const OkayRobot::DenavitHartenberg dh;
    const Bounds bounds;
};

class Description {
public:
    Description(const std::vector<JointDescription> joints)
        : joints_(joints) { };

    bool pose_is_valid(const OkayRobot::Pose pose) const;

    float upper_bound(const int& index) const;
    float lower_bound(const int& index) const;

    float dh_a(const int& index) const;
    float dh_d(const int& index) const;
    float dh_alpha(const int& index) const;
    float dh_theta(const int& index) const;
    const DenavitHartenberg dh(const int& index) const;

    int length() const;

private:
    const std::vector<JointDescription> joints_;
};
}