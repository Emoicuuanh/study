#!/bin/bash
# Xem cam bien truc quan (chay bang ROS2 he thong, python3.12)
#
# use_sim_time:=true BAT BUOC. Ly do:
#   Isaac Sim dong dau thoi gian bang THOI GIAN MO PHONG (bat dau tu 0),
#   con RViz mac dinh dung THOI GIAN HE THONG (epoch, so rat lon).
#   Lech nhau -> RViz coi moi du lieu la "den tu qua khu", vut bo het,
#   ngap tran canh bao TF_OLD_DATA va khong ve duoc gi.
#
#   Quy tac chung: MOI node ROS2 lam viec voi mo phong deu phai bat
#   use_sim_time, va mo phong phai phat /clock.
unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
exec rviz2 -d "$(dirname "$0")/src/g1_bringup/rviz/g1_sensors.rviz" \
     --ros-args -p use_sim_time:=true
