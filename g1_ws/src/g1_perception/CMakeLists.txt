cmake_minimum_required(VERSION 3.8)
project(g1_perception)

find_package(ament_cmake REQUIRED)

# Package nay chi chua launch + config (khong co ma nguon C++/Python).
# Cac node thuc su den tu pointcloud_to_laserscan va slam_toolbox.
install(DIRECTORY launch config
        DESTINATION share/${PROJECT_NAME})

# Node Python rieng cua ta. Dung PROGRAMS (khong phai FILES) de giu bit
# thuc thi - thieu no thi ros2 run bao "executable not found".
install(PROGRAMS scripts/stabilize_frame.py
                 scripts/lidar_add_fields.py
                 scripts/lio_eval.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
