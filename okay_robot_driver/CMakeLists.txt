cmake_minimum_required(VERSION 3.8)
project(okay_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)

set(SOURCES src/run_gamepad_node.cpp src/gamepad_node.cpp src/gamepad.cpp)
add_executable(gamepad_node ${SOURCES})

target_include_directories(
  gamepad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(gamepad_node okay_robot_msgs rclcpp)
target_link_libraries(gamepad_node SDL2)

install(TARGETS gamepad_node DESTINATION lib/${PROJECT_NAME})

ament_package()
