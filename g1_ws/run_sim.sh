#!/bin/bash
# Mo phong Isaac Sim (conda isaaclab, python3.11)
#
# === SO THREAD: 6, KHONG PHAI 20 (mac dinh cua Isaac Sim) ===
# Isaac Sim mac dinh tao 20 thread tasking + 20 thread TBB (= so loi may).
# Voi mot robot 12 khop thi do la QUA NHIEU: 40 thread tranh nhau 20 loi,
# phan lon thoi gian dung de dong bo va chuyen ngu canh chu khong phai
# tinh toan (oversubscription).
#
# Da do bang isaac/tests/bench_threads.sh (moi cau hinh 50 giay thuc):
#     threadCount   CPU     GPU    toc do mo phong
#        20        503%     52%      0.78x real-time    <- mac dinh
#         8        265%     55%      0.80x
#         6        222%     54%      1.00x              <- CHON
#         4        180%     51%      0.98x
# Giam 20 -> 6: CPU giam 56% VA mo phong chay NHANH HON.
# GPU giu nguyen ~52% o moi cau hinh -> GPU dang cho CPU, khong phai nut co chai.
#
# Muon doi: dat bien SIM_THREADS truoc khi chay, vd
#     SIM_THREADS=4 ./run_sim.sh --gait rl ...
cd "$(dirname "$0")/isaac"
unset VIRTUAL_ENV PYTHONPATH
export OMNI_KIT_ACCEPT_EULA=Y
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

THREADS="${SIM_THREADS:-6}"

exec ~/miniconda3/envs/isaaclab/bin/python run_g1_sim.py \
    --/plugins/carb.tasking.plugin/threadCount=$THREADS \
    --/plugins/omni.tbb.globalcontrol/maxThreadCount=$THREADS \
    "$@"
