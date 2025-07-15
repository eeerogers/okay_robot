cmake_minimum_required(VERSION 3.8)
project(okay_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(okay_robot_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

set(SOURCES src/run_control_node.cpp src/robot_control_node.cpp)
add_executable(control_robot_node ${SOURCES})

target_include_directories(
  control_robot_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(control_robot_node okay_robot_msgs geometry_msgs rclcpp)
target_link_libraries(control_robot_node Eigen3::Eigen)

install(TARGETS control_robot_node DESTINATION lib/${PROJECT_NAME})

ament_package()
