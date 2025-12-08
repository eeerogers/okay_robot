#include "okay_robot_common/timer/scope_timer.hpp"

ScopeTimer::~ScopeTimer()
{
    auto end_time = std::chrono::steady_clock::now();
    auto time_ms = std::chrono::duration<double, std::milli>(end_time - this->start_time_).count();

    if (this->timing_data_ != nullptr)
        this->timing_data_->add_time(this->name_, time_ms);

    // TODO: print out time on exit?
}