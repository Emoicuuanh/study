#!/usr/bin/env bash
# RViz cho CHANG A - Fixed Frame = map, co hien /map va /scan.
# Khac run_rviz.sh (Fixed Frame = odom, chi xem cam bien tho).
set -e
cd "$(dirname "$0")"

unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec rviz2 -d install/g1_bringup/share/g1_bringup/rviz/g1_slam.rviz \
  --ros-args -p use_sim_time:=true
