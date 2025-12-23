#include "okay_robot_view/time_series_data.hpp"

TimeSeriesData::TimeSeriesData(
    std::vector<std::shared_ptr<okay_robot_msgs::msg::ServoBusObservation>> observations)
{
    for (auto& obs : observations) {
        this->time.push_back(obs->observations[0].time);

        for (auto& ob : obs->observations) {
            // position
            if (this->positions.find(ob.id) != this->positions.end())
                this->positions[ob.id].push_back(ob.position);
            else
                this->positions[ob.id] = std::vector<float>({ ob.position });

            // speed
            if (this->speeds.find(ob.id) != this->speeds.end())
                this->speeds[ob.id].push_back(ob.speed);
            else
                this->speeds[ob.id] = std::vector<float>({ ob.speed });

            // load
            if (this->loads.find(ob.id) != this->loads.end())
                this->loads[ob.id].push_back(ob.load);
            else
                this->loads[ob.id] = std::vector<float>({ ob.load });

            // voltage
            if (this->voltages.find(ob.id) != this->voltages.end())
                this->voltages[ob.id].push_back(ob.voltage);
            else
                this->voltages[ob.id] = std::vector<float>({ ob.voltage });

            // temperature
            if (this->temperatures.find(ob.id) != this->temperatures.end())
                this->temperatures[ob.id].push_back(ob.temperature);
            else
                this->temperatures[ob.id] = std::vector<float>({ ob.temperature });

            // current
            if (this->currents.find(ob.id) != this->currents.end())
                this->currents[ob.id].push_back(ob.current);
            else
                this->currents[ob.id] = std::vector<float>({ ob.current });
        }
    }
}