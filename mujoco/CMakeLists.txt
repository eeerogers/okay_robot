cmake_minimum_required(VERSION 3.10)
project(MyProject)

set(CMAKE_CXX_STANDARD 17)

set(MUJOCO_PATH /opt/mujoco-3.3.2)

find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

include_directories(include ${MUJOCO_PATH}/include)
link_directories(${MUJOCO_PATH}/lib)

# Add source files
set(SOURCES main.cpp src/mujoco_gui.cpp)
add_executable(my_app ${SOURCES})

target_link_libraries(my_app mujoco glfw OpenGL::GL)