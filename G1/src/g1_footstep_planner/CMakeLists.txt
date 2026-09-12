cmake_minimum_required(VERSION 3.8)
project(g1_footstep_planner)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Generate Action Interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateFootstep.action"
  DEPENDENCIES geometry_msgs
)

# Include directories
include_directories(include)

# Libraries
add_library(footstep_calculator_lib STATIC src/footstep_calculator.cpp)
target_include_directories(footstep_calculator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Executables
add_executable(footstep_planner_action_node src/footstep_planner_action_node.cpp)
target_link_libraries(footstep_planner_action_node
  footstep_calculator_lib
)

ament_target_dependencies(footstep_planner_action_node
  rclcpp
  rclcpp_action
  geometry_msgs
  std_msgs
  nav_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
)

# Link generated action interface to executable
rosidl_target_interfaces(footstep_planner_action_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp"
)

# Install Targets & Directories
install(TARGETS
  footstep_calculator_lib
  footstep_planner_action_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(footstep_calculator_lib)
ament_export_dependencies(rclcpp rclcpp_action geometry_msgs std_msgs tf2 tf2_ros)

ament_package()
