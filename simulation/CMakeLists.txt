cmake_minimum_required(VERSION 3.8)
project(simulation)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(ros_gz REQUIRED)
find_package(ros_gz_interfaces REQUIRED)
find_package(common_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

# Add executables
add_executable(perfect_perception src/perfect_perception.cpp)
ament_target_dependencies(perfect_perception
  rclcpp
  std_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  ros_gz_interfaces
  common_msgs
)

add_executable(perfect_SLAM src/perfect_SLAM.cpp)
ament_target_dependencies(perfect_SLAM
  rclcpp
  std_msgs
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  ros_gz_interfaces
  common_msgs
)

add_executable(ackermann_to_speed_steer src/ackermann_to_speed_steer.cpp)
ament_target_dependencies(ackermann_to_speed_steer
  rclcpp
  std_msgs
  ackermann_msgs
  sensor_msgs
)

add_executable(perfect_path src/perfect_path.cpp)
ament_target_dependencies(perfect_path
  rclcpp
  nav_msgs
  geometry_msgs
  visualization_msgs
  tf2_ros
  tf2_geometry_msgs
  ament_index_cpp
)

# Install executables
install(TARGETS
  perfect_perception
  perfect_SLAM
  ackermann_to_speed_steer
  perfect_path
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY config launch models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()