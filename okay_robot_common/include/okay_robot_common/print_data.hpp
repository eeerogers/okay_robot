#pragma once

#include <numeric>
#include <stdint.h>
#include <string>
#include <vector>

void print_message(std::vector<uint8_t>& message);

template <typename T> std::string vec_to_string(const std::vector<T>& vec)
{
    std::string as_string = std::accumulate(
        vec.begin(), vec.end(), std::string(), [](const std::string& s, const T& n) {
            return s + (s.empty() ? "" : " ") + std::to_string(n);
        });

    return as_string;
}