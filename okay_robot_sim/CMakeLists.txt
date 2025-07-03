cmake_minimum_required(VERSION 3.8)
project(okay_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

set(MUJOCO_PATH /opt/mujoco-3.3.2)

include_directories(include ${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

set(SOURCES src/run_sim_node.cpp src/mujoco_sim_node.cpp)
add_executable(mujoco_sim_node ${SOURCES})
ament_target_dependencies(mujoco_sim_node rclcpp std_msgs)
target_link_libraries(mujoco_sim_node ${MUJOCO_PATH}/lib/libmujoco.so)

install(TARGETS mujoco_sim_node DESTINATION lib/${PROJECT_NAME})

ament_package()
