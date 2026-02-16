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
    void addTime(std::string& name, double& time_ms);
    void summarizeData() const;

private:
    std::unordered_map<std::string, TimeDataPoint> data_;
};