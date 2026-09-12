cmake_minimum_required(VERSION 3.5)
project(minkindr)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)

# Header-only library
add_library(${PROJECT_NAME} INTERFACE)
add_library(${PROJECT_NAME}::${PROJECT_NAME} ALIAS ${PROJECT_NAME})

target_include_directories(${PROJECT_NAME} INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} INTERFACE Eigen3::Eigen)

# Install targets and headers
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
)

install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME})
ament_export_include_directories(include)
ament_export_dependencies(Eigen3)

ament_package()




