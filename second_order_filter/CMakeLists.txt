cmake_minimum_required(VERSION 3.8)
project(second_order_filter)

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

# Add executable
add_executable(second_order_filter_node src/second_order_filter_node.cpp)
include_directories(include)

ament_target_dependencies(second_order_filter_node
  rclcpp
  geometry_msgs
)

# Install executable
install(TARGETS
  second_order_filter_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
