cmake_minimum_required(VERSION 3.8)
project(okay_robot_minimal_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

include_directories(include)
add_executable(minimal_node src/run_minimal_node.cpp src/minimal_node.cpp)
ament_target_dependencies(minimal_node rclcpp std_msgs)

install(TARGETS minimal_node DESTINATION lib/${PROJECT_NAME})

ament_package()
