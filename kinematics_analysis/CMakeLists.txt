cmake_minimum_required(VERSION 3.8)
project(kinematics_analysis)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Add executable
add_executable(kinematics_test_node src/kinematics_test_node.cpp)
include_directories(include)


ament_target_dependencies(kinematics_test_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  rosbag2_cpp
)


# Install executable
install(TARGETS
  kinematics_test_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
