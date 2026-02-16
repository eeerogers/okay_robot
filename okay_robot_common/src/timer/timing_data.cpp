#include <iostream>

#include "okay_robot_common/timer/timing_data.hpp"

void TimingData::addTime(std::string& name, double& time_ms)
{
    if (this->data_.find(name) == this->data_.end()) {
        this->data_[name] = TimeDataPoint({ time_ms, 1 });
    }

    else {
        this->data_[name].time_ms += time_ms;
        this->data_[name].counts++;
    }
}

void TimingData::summarizeData() const
{
    if (this->data_.size() == 0) {
        std::cout << "No data captured" << std::endl;
        return;
    }

    std::cout << "data summary: " << std::endl;
    for (const auto& data : this->data_) {
        std::cout << data.first << ":" << std::endl;
        std::cout << "\ttotal time:   " << data.second.time_ms << "ms" << std::endl;
        std::cout << "\tcount:        " << data.second.counts << std::endl;
        std::cout << "\taverage time: " << data.second.time_ms / data.second.counts << "ms"
                  << std::endl;
    }
}