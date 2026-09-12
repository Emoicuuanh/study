#!/usr/bin/env bash
# Build workspace ROS2 cua G1.
#
# HAI CAI BAY DA GAP, ca hai deu do MOI TRUONG PYTHON:
#
# 1. colcon khong co trong apt (may nay chua cai python3-colcon-*), nen no
#    duoc cai bang pip vao ~/.local/bin. Phai them vao PATH.
#
# 2. CMake tu chon `python3` DAU TIEN thay tren PATH. Tren may nay do la
#    /home/hungvd/miniconda3/bin/python3 - con nay KHONG co catkin_pkg nen
#    ament_cmake_core chet ngay:
#        ModuleNotFoundError: No module named 'catkin_pkg'
#    Cach chua: don PATH sach (bo miniconda) VA chi dinh thang
#    -DPython3_EXECUTABLE=/usr/bin/python3
#    Ngoai ra phai `unset VIRTUAL_ENV PYTHONPATH` vi ~/study/.venv la Python
#    3.13, con ROS2 Jazzy can 3.12.
set -e
cd "$(dirname "$0")"

unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
export PATH="$HOME/.local/bin:/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin"

colcon build --symlink-install \
  --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3 "$@"

echo
echo ">>> Build xong. Cac terminal khac hay chay:"
echo "    source $(pwd)/install/setup.bash"
