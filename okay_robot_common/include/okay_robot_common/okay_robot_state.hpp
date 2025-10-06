#pragma once

#include <Eigen/Dense>
#include <chrono>

class OkayRobotState {
public:
    OkayRobotState(
        std::chrono::steady_clock::time_point& t, Eigen::VectorXd& pos, Eigen::VectorXd& vel)
        : time(t)
        , joint_positions(pos)
        , joint_velocities(vel) { };

    std::chrono::steady_clock::time_point time;
    Eigen::VectorXd joint_positions;
    Eigen::VectorXd joint_velocities;
};

class OkayRobotGoal {
public:
    OkayRobotGoal();

    Eigen::VectorXd joint_positions;
};