#pragma once

#include <chrono>
#include <string>

#include "okay_robot_common/timer/timing_data.hpp"

class ScopeTimer {
public:
    ScopeTimer(std::string timer_name)
        : name_(timer_name)
        , start_time_(std::chrono::steady_clock::now()) { };
    ScopeTimer(std::string timer_name, TimingData* timing_data)
        : name_(timer_name)
        , timing_data_(timing_data)
        , start_time_(std::chrono::steady_clock::now()) { };
    ~ScopeTimer();

private:
    std::string name_;
    TimingData* timing_data_ = nullptr;
    std::chrono::steady_clock::time_point start_time_;
};