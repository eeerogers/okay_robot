cmake_minimum_required(VERSION 3.8)
project(okay_robot_gamepad)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(okay_robot_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(gamepad_node ${SOURCES})

ament_target_dependencies(gamepad_node okay_robot_msgs okay_robot_common rclcpp)
target_link_libraries(gamepad_node SDL2)
target_include_directories(
  gamepad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS gamepad_node DESTINATION lib/${PROJECT_NAME})

ament_package()
