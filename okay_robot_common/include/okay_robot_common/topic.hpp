#pragma once

namespace OkayRobot {

struct OkayRobotTopic {
    static constexpr char* GAMEPAD = "gamepad";
    static constexpr char* SERVO_BUS_COMMAND = "servo_bus_command";
    static constexpr char* SERVO_BUS_OBSERVATION = "servo_bus_observation";
    static constexpr char* OKAY_ROBOT_GOAL = "okay_robot_goal";
    static constexpr char* OKAY_ROBOT_GOAL_TWIST = "okay_robot_goal_twist";
};
}