#pragma once

struct TidyBotState {
    std::array<float, 7> joints;
    float gripper;

    float x_pos;
    float y_pos;
    float theta;
};

// TODO: turn this into base and arm transforms
struct TidyBotGoal {
    float x;
    float y;
    float z;

    float r;
    float p;
    float yaw; // TODO: make this not stupid
};