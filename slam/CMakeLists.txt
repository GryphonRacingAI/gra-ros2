cmake_minimum_required(VERSION 3.8)
project(slam)

find_package(ament_cmake REQUIRED)
find_package(ament_python REQUIRED)
# We specifically use find_package for all ROS dependencies to ensure they are available
find_package(rclpy REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(message_filters REQUIRED)
find_package(common_msgs REQUIRED)

# This command executes setup.py and installs the Python package correctly.
ament_python_install_package(${PROJECT_NAME})

# Install generated console scripts into lib/slam
install(PROGRAMS
  ${PROJECT_NAME}/bin/fastslam_node
  DESTINATION lib/${PROJECT_NAME}
)

# Export dependencies so downstream packages can correctly find the necessary ROS interfaces
ament_export_dependencies(
  rclpy
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  message_filters
  common_msgs
)

ament_package()
