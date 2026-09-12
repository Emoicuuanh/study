#!/bin/bash
# Dieu khien robot bang ban phim - phat /cmd_vel cho Isaac Sim.
#
# Chay o TERMINAL RIENG, sau khi mo phong da khoi dong voi co --teleop:
#     ./run_sim.sh --gait rl --teleop
#
#   u  i  o        i = tien        o = tien + xoay phai
#   j  k  l        j = xoay trai   k = DUNG    l = xoay phai
#   m  ,  .        , = lui
#   q/z  w/x  e/c  tang/giam toc do
#
# HANH VI QUAN TRONG - "chot lenh", khong phai "giu phim":
#   teleop_twist_keyboard dung sys.stdin.read(1) CHAN cho den khi co phim,
#   roi phat DUNG MOT message. Khong co vong lap phat lai.
#   Con ROS2SubscribeTwist ben Isaac Sim GIU gia tri cuoi nhan duoc.
#   => Bam 'i' MOT LAN thi robot di tien MAI. Muon dung phai bam 'k'.
#   Rat de tuong la hong khi thay robot khong dung sau khi tha phim.
#
# Toc do dat 0.4 m/s cho khop policy di bo cua G1 (train quanh 0.5 m/s).
unset VIRTUAL_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "----------------------------------------------------------"
echo " LUU Y: bam MOT LAN la robot di MAI. Bam 'k' de dung."
echo " Terminal nay phai dang duoc focus moi nhan duoc phim."
echo "----------------------------------------------------------"
exec ros2 run teleop_twist_keyboard teleop_twist_keyboard \
    --ros-args -p speed:=0.4 -p turn:=0.4 -r __ns:=/
