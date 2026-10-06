#!/usr/bin/env bash
# Chay node che do bong ban C++. CHI DOC rt/lowstate, KHONG gui lenh nao.
#
#   bash G1/src/g1_wbc_cpp/test/run_shadow.sh [--secs 60] [--csv ra.csv] ...
#
# Tu tim card mang o cung subnet voi robot: may nay co hai card wifi va chung
# doi cho mang cho nhau, ghi cung ten card thi hong im lang.
set -eo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
SUBNET="${G1_SUBNET:-192.168.123.}"

IFACE=""
for d in /sys/class/net/*/; do
  i=$(basename "$d")
  [ "$i" = "lo" ] && continue
  [ "$(cat "$d/carrier" 2>/dev/null)" = "1" ] || continue
  if ip -4 -br addr show "$i" 2>/dev/null | grep -q " ${SUBNET}"; then IFACE="$i"; break; fi
done
if [ -z "$IFACE" ]; then
  echo "khong card nao o mang robot ${SUBNET}x - robot da bat chua? wifi da noi chua?"
  ip -4 -br addr | grep -v '^lo'
  exit 2
fi
echo "card robot: $IFACE ($(ip -4 -br addr show "$IFACE" | awk '{print $3}'))"

# Chi can cho LD_LIBRARY_PATH (pinocchio/proxsuite nam trong /opt/ros). Node nay
# KHONG dung ROS nen khong dinh xung dot RMW voi SDK.
source /opt/ros/jazzy/setup.bash
source "$REPO/G1/install/setup.bash"
# BAT BUOC dat TRUOC duong dan cua ROS. /opt/ros/jazzy cung co libddsc.so.0, va
# LD_LIBRARY_PATH thang ca RPATH - nen libddscxx cua SDK (dung theo CycloneDDS
# 0.10.2-noshm) se goi vao libddsc cua ROS. Hai ban khac layout struct -> hong
# heap, bieu hien la "free(): invalid next size" sau vai chuc nghin goi.
export LD_LIBRARY_PATH="${UNITREE_SDK2_DIR:-$HOME/unitree_sdk2}/thirdparty/lib/x86_64:$LD_LIBRARY_PATH"

cd "$REPO"
exec "$REPO/G1/install/g1_wbc_cpp/lib/g1_wbc_cpp/shadow_node" "$IFACE" \
  --urdf "$REPO/G1/src/g1_description/urdf/g1_29dof.urdf" \
  --payload "$REPO/G1/src/g1_leg_odometry/config/payload.yaml" \
  "$@"
