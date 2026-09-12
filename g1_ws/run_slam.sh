#!/usr/bin/env bash
# CHANG A - chay SLAM 2D (pointcloud_to_laserscan + slam_toolbox).
#
# Thu tu khoi chay 3 terminal:
#   T1:  ./run_sim.sh --gait rl --teleop
#   T2:  ./run_slam.sh          <- file nay
#   T3:  ./run_rviz_slam.sh
#   T4:  ./run_teleop.sh        (dieu khien robot di quet ban do)
#
# LUU Y: phai doi terminal 1 in ra ">>> Chay ...s. Topic ROS2: ..." roi moi
# chay file nay. Neu chay truoc, slam_toolbox khoi tao khi chua co /clock ->
# no dung thoi gian 0 va bo moi quet den sau.
set -e
cd "$(dirname "$0")"

unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
source install/setup.bash

exec ros2 launch g1_perception slam_2d.launch.py "$@"
