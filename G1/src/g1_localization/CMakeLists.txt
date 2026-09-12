cmake_minimum_required(VERSION 3.8)
project(g1_localization)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O3)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(pcl_conversions REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS common io registration filters)

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

# ROS 2 NDT Localization Node
add_executable(ndt_localization_node
  src/ndt_localization_node.cpp
)
target_link_libraries(ndt_localization_node
  ${PCL_LIBRARIES}
)
ament_target_dependencies(ndt_localization_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)

# Install targets
install(TARGETS
  ndt_localization_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz scripts
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
