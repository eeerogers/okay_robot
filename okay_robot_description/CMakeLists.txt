cmake_minimum_required(VERSION 3.8)
project(okay_robot_description)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)

add_library(okay_robot_description INTERFACE)
target_include_directories(okay_robot_description INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(okay_robot_description INTERFACE ament_index_cpp::ament_index_cpp)

install(
  DIRECTORY urdf/stanford_tidybot urdf/kinova_gen3 urdf/low_cost_robot_arm urdf/unitree_g1 urdf/
  DESTINATION share/${PROJECT_NAME}
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS okay_robot_description
  EXPORT export_okay_robot_description
)

install(
  EXPORT export_okay_robot_description
  NAMESPACE okay_robot_description::
  DESTINATION lib/cmake/okay_robot_description
)

ament_export_dependencies(ament_index_cpp)
ament_export_targets(export_okay_robot_description)
ament_package()
