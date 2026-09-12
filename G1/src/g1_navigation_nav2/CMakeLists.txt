cmake_minimum_required(VERSION 3.5)
project(g1_navigation_nav2)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

# Install directories
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/g1_safety_watchdog.py
    scripts/g1_twist_mux.py
  DESTINATION lib/${PROJECT_NAME}
)


ament_package()

