#pragma once

#include <Eigen/Dense>
#include <chrono>

class OkayRobotObservation {
public:
    OkayRobotObservation(const std::chrono::steady_clock::time_point t, const Eigen::VectorXd pos,
        const Eigen::VectorXd vel)
        : time(t)
        , joint_positions(pos)
        , joint_velocities(vel) { };

    const std::chrono::steady_clock::time_point time;

    const Eigen::VectorXd joint_positions;
    const Eigen::VectorXd joint_velocities;
};

class OkayRobotGoal {
public:
    OkayRobotGoal();

    Eigen::VectorXd joint_positions;
};