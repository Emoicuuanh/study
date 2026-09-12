cmake_minimum_required(VERSION 3.8)
project(aruco_detector)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

# Generate custom ROS 2 messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/ArucoMarker.msg"
  "msg/ArucoMarkerArray.msg"
  DEPENDENCIES std_msgs geometry_msgs
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  aruco_detector/aruco_detector_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch and config directories
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
