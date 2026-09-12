#!/usr/bin/env bash
# Tim so thread toi uu: do CPU VA toc do mo phong cung luc.
#
#   SCRATCH=/tmp bash isaac/tests/bench_threads.sh
#
# Chi do CPU thi de bi lua: giam thread lam CPU giam nhung neu mo phong
# cham lai thi khong duoc gi. Chi so that su can la
#     toc do = thoi gian MO PHONG troi / thoi gian THUC troi
# 1.0 = chay dung real-time. Duoi 1.0 = cham hon thuc te.
set -u
cd "$(dirname "$0")/../.."
S="${SCRATCH:-/tmp}/thr"
mkdir -p "$S"
WALL=50           # do trong 50 giay thuc

for N in 20 8 6 4; do
  f="$S/n$N.log"
  nohup ./run_sim.sh --gait rl --no-camera --headless --scene room \
      --duration 600 --vx 0.4 --omega 0.15 \
      --/plugins/carb.tasking.plugin/threadCount=$N \
      --/plugins/omni.tbb.globalcontrol/maxThreadCount=$N > "$f" 2>&1 &
  for _ in $(seq 1 40); do
    grep -q "Topic ROS2" "$f" 2>/dev/null && break
    sleep 3
  done
  sleep 8                                  # cho on dinh
  pid=$(pgrep -f "[i]saaclab/bin/python run_g1_sim" | head -1)
  t0=$(grep -oP '^>>> t=\s*\K[0-9]+' "$f" | tail -1); t0=${t0:-0}
  w0=$SECONDS

  # do CPU trong luc cho
  cpu=$( for _ in 1 2 3 4; do
           top -b -n 2 -d 2 -p "$pid" 2>/dev/null | awk '/python/{v=$9} END{print v}'
         done | awk '{s+=$1;n++} END{printf "%.0f", (n?s/n:0)}' )
  gpu=$(nvidia-smi --query-gpu=utilization.gpu --format=csv,noheader,nounits)
  while [ $((SECONDS-w0)) -lt $WALL ]; do sleep 2; done

  t1=$(grep -oP '^>>> t=\s*\K[0-9]+' "$f" | tail -1); t1=${t1:-0}
  wall=$((SECONDS-w0))
  speed=$(echo "scale=2; ($t1-$t0)/$wall" | bc)
  printf "threadCount=%-3s CPU %5s%%   GPU %3s%%   toc do %sx real-time  (sim %ss / thuc %ss)\n" \
         "$N" "$cpu" "$gpu" "$speed" "$((t1-t0))" "$wall"

  pkill -f "[i]saaclab/bin/python run_g1_sim" 2>/dev/null
  sleep 6
done
