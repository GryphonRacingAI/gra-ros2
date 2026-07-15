cmake_minimum_required(VERSION 3.8)
project(control)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)  

ament_python_install_package(${PROJECT_NAME})

install(PROGRAMS
	control/mppi_ros_modified.py
	control/pure_pursuit.py
	DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config/
	DESTINATION share/${PROJECT_NAME}/config
)

ament_package()