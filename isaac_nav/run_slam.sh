#!/bin/bash
# TERMINAL 2 - Chay SLAM (ROS2 he thong, python3.12) + RViz2
unset VIRTUAL_ENV PYTHONPATH        # tranh venv python3.13 chen vao ROS2
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec ros2 launch "$(dirname "$0")/ros2/slam.launch.py"
