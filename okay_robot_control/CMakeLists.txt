cmake_minimum_required(VERSION 3.8)
project(okay_robot_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)

find_package(geometry_msgs REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(okay_robot_common REQUIRED)

set(
  SOURCES
  src/run_control_node.cpp
  src/robot_control_node.cpp
  src/kinematics.cpp
  src/control/direct_controller.cpp
)
add_executable(robot_control_node ${SOURCES})

target_include_directories(
  robot_control_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIRS}
)

ament_target_dependencies(robot_control_node okay_robot_msgs okay_robot_common geometry_msgs rclcpp)
target_link_libraries(robot_control_node Eigen3::Eigen)

install(TARGETS robot_control_node DESTINATION lib/${PROJECT_NAME})

ament_package()
