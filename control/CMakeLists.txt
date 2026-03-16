cmake_minimum_required(VERSION 3.8)
project(control)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ackermann_msgs REQUIRED)

install(PROGRAMS
  scripts/mppi_ros_modified.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
