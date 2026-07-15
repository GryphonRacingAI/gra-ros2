cmake_minimum_required(VERSION 3.5)
project(fsai_api)

# C++ standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ackermann_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Message files
set(msg_files
    "msg/VCU2AI.msg"
)

# Message generation
rosidl_generate_interfaces(${PROJECT_NAME}
    ${msg_files}
    DEPENDENCIES std_msgs
)

# Build FS-AI_API C library
add_library(fs_ai_api STATIC
    src/fs-ai_api.c
    src/can.c
)
target_include_directories(fs_ai_api 
    PUBLIC include
    PRIVATE src include/fsai_api
)

# Add ackermann_can executable
add_executable(ackermann_can src/ackermann_can.cpp)
ament_target_dependencies(ackermann_can rclcpp ackermann_msgs std_msgs)
target_link_libraries(ackermann_can fs_ai_api)

# Link generated message type support libraries
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")
target_link_libraries(ackermann_can ${cpp_typesupport_target})

# Install targets
install(TARGETS
    fs_ai_api
    ackermann_can
    DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
    scripts/wheel_speed_controller.py
    scripts/vcu_sim.py
    DESTINATION lib/${PROJECT_NAME}
)

# Export rosidl runtime dependencies
ament_export_dependencies(rclcpp ackermann_msgs std_msgs rosidl_default_runtime)

ament_package()