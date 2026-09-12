#!/usr/bin/env bash
# Do CPU/GPU cua 4 to hop cau hinh, cung dieu kien.
#
#   bash isaac/tests/bench_cpu.sh
#
# Moi lan chay 60s sim, do CPU trung binh trong 20s giua (bo qua giai doan
# khoi dong vi luc do CPU tang vot do nap USD/shader).
set -u
cd "$(dirname "$0")/../.."
S="${SCRATCH:-/tmp}/bench"
mkdir -p "$S"

run_one() {
  local name="$1"; shift
  echo "=============================================================="
  echo ">>> $name"
  echo "    co: $*"
  nohup ./run_sim.sh --gait rl --no-camera --headless --scene room \
        --duration 90 --vx 0.4 --omega 0.15 "$@" > "$S/$name.log" 2>&1 &
  # doi san sang
  for _ in $(seq 1 40); do
    grep -q "Topic ROS2" "$S/$name.log" 2>/dev/null && break
    pgrep -f "run_g1_sim.py" >/dev/null || break
    sleep 3
  done
  sleep 12          # cho on dinh
  local pid; pid=$(pgrep -f "isaaclab/bin/python run_g1_sim.py" | head -1)
  if [ -z "$pid" ]; then echo "    LOI: sim khong chay"; return; fi

  # CPU trung binh qua 5 mau cach nhau 3s
  local tot=0 n=0
  for _ in $(seq 1 5); do
    local c; c=$(top -b -n 2 -d 2 -p "$pid" 2>/dev/null | awk '/python/{v=$9} END{print v}')
    [ -n "$c" ] && { tot=$(echo "$tot + $c" | bc); n=$((n+1)); }
  done
  local gpu; gpu=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits)
  grep -E "vat ly:|drive_mode" "$S/$name.log" | sed 's/^/    /'
  echo "    CPU trung binh : $(echo "scale=1; $tot/$n" | bc) %  (tren 20 loi = 2000%)"
  echo "    GPU            : $gpu %"
  # toc do mo phong: t cuoi / thoi gian thuc
  grep -E "^>>> t=" "$S/$name.log" | tail -1 | sed 's/^/    /'

  pkill -f "isaaclab/bin/python run_g1_sim.py" 2>/dev/null
  sleep 6
}

run_one "1_cpu_torque"
run_one "2_cpu_position" --drive position
run_one "3_gpu_torque"   --gpu-physics
run_one "4_gpu_position" --gpu-physics --drive position

echo "=============================================================="
echo "log day du: $S/"
