#!/bin/bash
# TERMINAL 3 - Xem truc quan
unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "$(dirname "$0")/ros2/g1_nav.rviz"
