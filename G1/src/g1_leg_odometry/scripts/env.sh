# Nap moi truong cho g1_leg_odometry.  Dung:  source scripts/env.sh
#
# Vi sao can: cac phu thuoc (pinocchio, unitree_sdk2py, cyclonedds) nam trong
# venv /home/dung/study/.venv-ros chu khong phai python he thong. Package da
# duoc build BANG python cua venv do, nen shebang cua node tro thang vao day -
# khong can activate venv, chi can source ROS + install la chay duoc.
source /opt/ros/jazzy/setup.bash
# Suy ra duong dan tu vi tri file nay, KHONG ghi cung - repo da tung bi doi ten
# tu /home/dung/study sang /home/dung/STUDY va lam hong het duong dan cung.
_ENV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
G1_WS="$(cd "$_ENV_DIR/../../.." && pwd)"          # .../G1
REPO="$(cd "$G1_WS/.." && pwd)"
source "$G1_WS/install/setup.bash"
echo "ROS_DISTRO=$ROS_DISTRO | node dung $(head -1 "$G1_WS/install/g1_leg_odometry/lib/g1_leg_odometry/leg_odometry_node" | sed 's|#!||')"

# --- ROS <-> robot ---
# CANH BAO: KHONG bat RMW_IMPLEMENTATION=rmw_cyclonedds_cpp cho cac node dung
# unitree_sdk2py (leg_odometry_node, wbc_shadow_node).
#
# Ly do: robot phat bang CycloneDDS domain 0. Neu ROS cung dung CycloneDDS thi
# trong MOT tien trinh se co hai ben cung muon tao domain 0 voi hai cau hinh
# khac nhau -> CycloneDDS tu choi:
#     ros truoc : "[ChannelFactory] create domain error"
#     sdk truoc : "rmw_create_node: failed to create domain, Precondition Not Met"
# Da thu ca hai thu tu, deu hong.
#
# Vi vay: node dung SDK -> de RMW mac dinh (FastDDS). Chung noi chuyen voi robot
# qua SDK chu khong qua ROS, nen khong can thay topic DDS cua robot.
#
# Neu CAN ROS thay truc tiep /dog_imu_raw, /dog_odom, /utlidar/... (vd de chay
# g1_state_estimator hoac FAST-LIO) thi bat trong tien trinh RIENG, KHONG dung SDK:
#     source .../env.sh ros_bridge
#
# Huong di dung ve lau dai: sinh goi ROS message cho kieu unitree_hg (cach
# unitree_ros2 chinh thuc lam), roi subscribe rt/lowstate nhu ROS topic binh
# thuong - bo han SDK khoi tien trinh, het xung dot.

export G1_IFACE="${G1_IFACE:-wlp0s20f3}"

if [ "$1" = "ros_bridge" ]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$G1_IFACE\"/></Interfaces></General></Domain></CycloneDDS>"
  echo "CHE DO ros_bridge: RMW=$RMW_IMPLEMENTATION | card=$G1_IFACE"
  echo "  -> KHONG chay node dung unitree_sdk2py trong shell nay"
  echo "  -> doi RMW xong phai 'ros2 daemon stop'"
else
  unset RMW_IMPLEMENTATION CYCLONEDDS_URI
  echo "RMW mac dinh (FastDDS) | card robot=$G1_IFACE"
  echo "  -> hop cho leg_odometry_node / wbc_shadow_node"
fi

# BAY khac: 'ros2 topic hz' KHONG chay voi publisher DDS thuan cua robot
#   (Node name: _CREATED_BY_BARE_DDS_APP_, type hash INVALID).
#   Dung 'ros2 topic echo --once --qos-reliability best_effort' hoac subscriber
#   that. Da do: dog_imu_raw 1042 Hz, dog_odom 1042 Hz, lidar 9.9 Hz.
