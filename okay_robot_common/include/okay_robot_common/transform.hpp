#include <Eigen/Dense>

class Transform {
public:
    Transform()
        : matrix(Eigen::Matrix4d::Identity()) { };
    Transform(Eigen::Matrix4d mat)
        : matrix(mat) { };

    Eigen::Matrix4d matrix;

    Transform forward(const Transform& other);
    Transform inverse();
};