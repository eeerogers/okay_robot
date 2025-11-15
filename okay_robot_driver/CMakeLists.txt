cmake_minimum_required(VERSION 3.8)
project(okay_robot_driver)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(okay_robot_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(SDL2 REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(LibSerial REQUIRED libserial)

include_directories(${LibSerial_INCLUDE_DIRS})
link_directories(${LibSerial_LIBRARY_DIRS})

add_executable(
  gamepad_node
  src/gamepad/run_gamepad_node.cpp
  src/gamepad/gamepad_node.cpp
  src/gamepad/gamepad.cpp
)
ament_target_dependencies(gamepad_node okay_robot_msgs okay_robot_common rclcpp)
target_link_libraries(gamepad_node SDL2)
target_include_directories(
  gamepad_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(
  servo_bus_node
  src/servo_bus/run_servo_bus_node.cpp
  src/servo_bus/servo_bus_node.cpp
  src/servo_bus/servo_bus.cpp
)
ament_target_dependencies(servo_bus_node okay_robot_msgs okay_robot_common rclcpp)
target_link_libraries(servo_bus_node ${LibSerial_LIBRARIES})
target_include_directories(
  servo_bus_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(TARGETS gamepad_node servo_bus_node DESTINATION lib/${PROJECT_NAME})

ament_package()
