cmake_minimum_required(VERSION 3.8)
project(okay_robot_view)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(OpenGL_GL_PREFERENCE GLVND)

find_package(ament_cmake REQUIRED)
find_package(okay_robot_msgs REQUIRED)
find_package(okay_robot_common REQUIRED)
find_package(rclcpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(glfw3 REQUIRED)

set(IMGUI_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/imgui)
set(IMPLOT_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/implot)

set(IMGUI_SOURCES
    ${IMGUI_DIR}/imgui.cpp
    ${IMGUI_DIR}/imgui_demo.cpp
    ${IMGUI_DIR}/imgui_draw.cpp
    ${IMGUI_DIR}/imgui_tables.cpp
    ${IMGUI_DIR}/imgui_widgets.cpp
    ${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
    ${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
)

set(IMPLOT_SOURCES
    ${IMPLOT_DIR}/implot.cpp
    ${IMPLOT_DIR}/implot_demo.cpp
    ${IMPLOT_DIR}/implot_items.cpp
)

file(GLOB_RECURSE SOURCES CONFIGURE_DEPENDS src/*.cpp)
add_executable(gui_app
    ${SOURCES}
    ${IMGUI_SOURCES}
    ${IMPLOT_SOURCES}
)

target_include_directories(
    gui_app PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${IMGUI_DIR}
    ${IMGUI_DIR}/backends
    ${IMPLOT_DIR}
)

target_compile_options(gui_app PRIVATE -Wno-unused-parameter)
ament_target_dependencies(gui_app okay_robot_msgs okay_robot_common rclcpp)
target_link_libraries(gui_app glfw OpenGL::GL)

install(TARGETS gui_app DESTINATION lib/${PROJECT_NAME})

ament_package()