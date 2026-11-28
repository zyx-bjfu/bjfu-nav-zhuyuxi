cmake_minimum_required(VERSION 3.8)
project(astar_planner)

# colcon does not set CMAKE_BUILD_TYPE by default, which leaves gcc at -O0 and
# makes any timing number meaningless. Default to Release for this package.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
  message(STATUS "astar_planner: CMAKE_BUILD_TYPE not set, defaulting to Release")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

add_library(astar_core SHARED src/astar_planner.cpp)
target_include_directories(astar_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(astar_core PUBLIC c_std_11 cxx_std_17)

add_executable(astar_node src/astar_node.cpp)
target_link_libraries(astar_node astar_core)
ament_target_dependencies(astar_node
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs)

install(TARGETS astar_core astar_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY include/ DESTINATION include)

ament_export_include_directories(include)
ament_export_libraries(astar_core)
ament_export_dependencies(rclcpp nav_msgs geometry_msgs visualization_msgs)

ament_package()
