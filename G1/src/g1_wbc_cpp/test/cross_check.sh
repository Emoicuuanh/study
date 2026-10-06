#!/usr/bin/env bash
# Doi chieu ban C++ voi ban Python tren test vector. Mot lenh, khong can robot.
#
#   bash G1/src/g1_wbc_cpp/test/cross_check.sh [CHUAN.txt]
#
# Mac dinh dung G1/data/test_vectors/sim_pipeline.txt (sinh tu MuJoCo). Khi da
# ghi duoc robot that thi truyen file chuan sinh tu ban ghi that vao day.
set -eo pipefail   # khong dung -u: setup.bash cua ROS doc bien chua dat
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO="$(cd "$HERE/../../../.." && pwd)"
GOLD="${1:-$REPO/G1/data/test_vectors/sim_pipeline.txt}"
OUT="$(mktemp -d)/cpp_out.txt"

if [ ! -f "$GOLD" ]; then
  echo "khong co file chuan: $GOLD"
  echo "sinh bang: .venv-real/bin/python G1/src/g1_wbc/test/test_record_replay_pipeline.py --keep"
  exit 2
fi

source /opt/ros/jazzy/setup.bash
source "$REPO/G1/install/setup.bash"
"$REPO/G1/install/g1_wbc_cpp/lib/g1_wbc_cpp/run_test_vectors" \
  "$GOLD" "$OUT" \
  "$REPO/G1/src/g1_description/urdf/g1_29dof.urdf" \
  "$REPO/G1/src/g1_leg_odometry/config/payload.yaml" \
  --bench 20
echo
"$REPO/.venv-real/bin/python" "$REPO/G1/src/g1_wbc/g1_wbc/test_vectors.py" check "$GOLD" "$OUT"
