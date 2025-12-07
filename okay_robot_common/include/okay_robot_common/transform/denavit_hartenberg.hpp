#pragma once

#include <Eigen/Dense>

namespace OkayRobot {
class DenavitHartenberg {
public:
    DenavitHartenberg(const float& a, const float& d, const float& alpha, const float& theta)
        : a(a)
        , d(d)
        , alpha(alpha)
        , theta(theta) { };

    const float a;
    const float d;
    const float alpha;
    const float theta;
};
}