cmake_minimum_required(VERSION 3.8)
project(okay_robot_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenGL_GL_PREFERENCE GLVND)
set(MUJOCO_PATH /opt/mujoco-3.3.2)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(okay_robot_description REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(SOURCES src/run_sim_node.cpp src/mujoco_sim_node.cpp src/mujoco_gui.cpp)
add_executable(mujoco_sim_node ${SOURCES})

target_include_directories(
  mujoco_sim_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
link_directories(${MUJOCO_PATH}/lib)

target_compile_options(mujoco_sim_node PRIVATE -Wno-unused-parameter)

ament_target_dependencies(mujoco_sim_node okay_robot_msgs okay_robot_description rclcpp)
target_link_libraries(mujoco_sim_node glfw OpenGL::GL ${MUJOCO_PATH}/lib/libmujoco.so)

install(TARGETS mujoco_sim_node DESTINATION lib/${PROJECT_NAME})

ament_package()
