#!/bin/bash
# TERMINAL 1 - Chay mo phong Isaac Sim (phat LiDAR + camera + odom + TF ra ROS2)
cd "$(dirname "$0")"
export OMNI_KIT_ACCEPT_EULA=Y
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
unset VIRTUAL_ENV PYTHONPATH        # tranh venv/ROS2 he thong chen vao Isaac Sim
exec ~/miniconda3/envs/isaaclab/bin/python step_e_full.py
