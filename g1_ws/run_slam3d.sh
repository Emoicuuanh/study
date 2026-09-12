#!/usr/bin/env bash
# CHANG B - SLAM 3D bang FAST-LIO.
#
# Thu tu khoi chay:
#   T1:  ./run_sim.sh --gait rl --no-camera --headless --scene room --teleop
#   T2:  ./run_slam3d.sh        <- file nay
#   T3:  ./run_rviz_slam3d.sh
#   T4:  ./run_teleop.sh
#
# Doi terminal 1 in ra ">>> Chay ...s. Topic ROS2: ... /imu" roi moi chay.
set -e
cd "$(dirname "$0")"

unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 launch g1_perception slam_3d.launch.py "$@"
