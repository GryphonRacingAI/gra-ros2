cmake_minimum_required(VERSION 3.8)
project(fastslam_ros)

find_package(ament_cmake REQUIRED)
find_package(python_cmake_module REQUIRED)

# This command executes setup.py and installs the Python package correctly.
ament_python_install_package(${PROJECT_NAME})

# Find and export dependencies so other packages can correctly link against messages
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)

ament_export_dependencies(
  rclpy
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  message_filters
)
