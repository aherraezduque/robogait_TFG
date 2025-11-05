cmake_minimum_required(VERSION 3.10)
project(actor_command_plugin)

# C++ standard
set(CMAKE_CXX_STANDARD 17)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ignition-plugin1 REQUIRED COMPONENTS loader register)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_msgs REQUIRED)

# Include directories
include_directories(
  include
)

# Plugin source
add_library(actor_command_system SHARED
  src/actor_command_system.cpp
)

ament_target_dependencies(actor_command_system
  ignition-plugin1
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  ignition-gazebo6
  ignition-math6
  tf2
  tf2_geometry_msgs
  custom_msgs
)

# Install target
install(TARGETS actor_command_system
  LIBRARY DESTINATION lib
)

install(DIRECTORY include/
  DESTINATION include/
)

ament_package()
