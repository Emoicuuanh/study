cmake_minimum_required(VERSION 3.5)
project(minkindr_conversions)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(gflags REQUIRED)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../../cmake")
find_package(glog REQUIRED)

find_package(minkindr REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(tf2_eigen REQUIRED)

add_library(${PROJECT_NAME} INTERFACE)

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${Eigen3_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${minkindr_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_eigen_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME} INTERFACE
  ${Eigen3_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  gflags
  glog::glog
  minkindr::minkindr
  ${tf2_LIBRARIES}
  ${tf2_ros_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${tf2_eigen_LIBRARIES}
)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(
  Eigen3
  geometry_msgs
  gflags
  glog
  minkindr
  tf2
  tf2_ros
  tf2_geometry_msgs
  tf2_eigen
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  
  ament_add_gtest(kindr_tf_test test/kindr_tf_test.cc)
  target_include_directories(kindr_tf_test PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${minkindr_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
  )
  target_link_libraries(kindr_tf_test ${PROJECT_NAME})

  ament_add_gtest(kindr_msg_test test/kindr_msg_test.cc)
  target_include_directories(kindr_msg_test PRIVATE
    ${Eigen3_INCLUDE_DIRS}
    ${geometry_msgs_INCLUDE_DIRS}
    ${minkindr_INCLUDE_DIRS}
    ${tf2_INCLUDE_DIRS}
    ${tf2_ros_INCLUDE_DIRS}
    ${tf2_geometry_msgs_INCLUDE_DIRS}
    ${tf2_eigen_INCLUDE_DIRS}
  )
  target_link_libraries(kindr_msg_test ${PROJECT_NAME})
endif()

ament_package()

