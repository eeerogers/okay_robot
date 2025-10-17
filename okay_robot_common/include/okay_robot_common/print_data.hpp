#pragma once

#include <stdint.h>
#include <string>
#include <vector>

void print_message(std::vector<uint8_t>& message);

template <typename T> std::string vec_to_string(std::vector<T>& vec);