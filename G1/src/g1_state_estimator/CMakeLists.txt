cmake_minimum_required(VERSION 3.8)
project(g1_state_estimator)

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
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters registration)
find_package(GTSAM REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate custom service messages
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/SaveMap.srv"
)


# Core library (Math and IEKF logic - independent of ROS 2)
add_library(estimator_core SHARED
  src/estimator.cpp
)
target_include_directories(estimator_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
)
target_link_libraries(estimator_core
  Eigen3::Eigen
)

# ROS 2 node executable
add_executable(state_estimator_node
  src/estimator_node.cpp
  src/main.cpp
)
target_include_directories(state_estimator_node PRIVATE
  include
)
target_link_libraries(state_estimator_node
  estimator_core
)
ament_target_dependencies(state_estimator_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf2
  tf2_ros
  tf2_eigen
)

# ROS 2 GTSAM Loop Closure Node
add_executable(gtsam_loop_closure_node
  src/gtsam_loop_closure_node.cpp
  src/gtsam_loop_closure_main.cpp
)
target_include_directories(gtsam_loop_closure_node PRIVATE
  include
  ${PCL_INCLUDE_DIRS}
  ${GTSAM_INCLUDE_DIR}
)
target_link_libraries(gtsam_loop_closure_node
  ${PCL_LIBRARIES}
  gtsam
)
target_compile_definitions(gtsam_loop_closure_node PRIVATE
  # Avoid potential boost/GTSAM compile warning issues
  BOOST_BIND_NO_PLACEHOLDERS
)
ament_target_dependencies(gtsam_loop_closure_node
  rclcpp
  sensor_msgs
  nav_msgs
  geometry_msgs
  std_srvs
  tf2
  tf2_ros
  tf2_eigen
  pcl_conversions
)
rosidl_target_interfaces(gtsam_loop_closure_node ${PROJECT_NAME} "rosidl_typesupport_cpp")

# ROS 2 Timestamp Corrector Node (Zero-copy C++ node)
add_executable(timestamp_corrector_node
  src/timestamp_corrector_node.cpp
)
target_include_directories(timestamp_corrector_node PRIVATE include)
ament_target_dependencies(timestamp_corrector_node
  rclcpp
  sensor_msgs
)

# Install targets
install(TARGETS
  estimator_core
  state_estimator_node
  gtsam_loop_closure_node
  timestamp_corrector_node
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/save_all_maps.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
