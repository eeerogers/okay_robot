#pragma once

#include <chrono>
#include <string>
#include <unordered_map>

struct TimeDataPoint {
    double time_ms;
    int counts;
};

class TimingData {
public:
    void add_time(std::string& name, double& time_ms);
    void summarize_data() const;

private:
    std::unordered_map<std::string, TimeDataPoint> data_;
};