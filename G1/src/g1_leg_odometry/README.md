# g1_leg_odometry

Van toc than cho Unitree G1 tinh tu **encoder + gyro + mo-men khop**, thay the
`/dog_odom` khi chuyen sang dieu khien low-level.

## Tai sao

`g1_state_estimator/src/estimator.cpp:97` `UpdateLeg()` lay van toc than tu
`/dog_odom` (`config/estimator.yaml:6`). Topic do la **SDK high-level sinh ra**,
tuc la do chinh bo dieu khien ma WBC low-level se tat. Khi chuyen sang
`rt/lowcmd`, nguon do hoac chet, hoac te hon la tra so cu ma IEKF van tin.
FAST-LIO thi chi 50-100Hz, qua cham cho vong dieu khien 500Hz.

## Nguyen ly

Chan dang tua (khong truot) => van toc chan bang 0:

    v_B = -(w_B x p_BF + J_BF * qdot_leg)

Khong co ma tran quay trong ve phai: **van toc than trong he than chi can gyro
va encoder**, khong can biet huong robot. Sai so huong cua IMU khong lot vao
phep do nay. Do dung la dai luong `UpdateLeg()` can (`leg.v_body`).

FK/Jacobian tinh voi than ghim tai goc, huong don vi - co y khong dung pose
that cua than, vi robot that khong biet pose do.

## Do duoc (doi chieu ground truth trong MuJoCo)

| | sach | co nhieu cam bien |
|---|---|---|
| v_body RMS (3 truc) | 2.64 mm/s | 13.86 mm/s |
| fz khi tua tinh, sai so RMS | 1.4 N | 1.5 N |

Bien do van toc that trong bai test: 257 mm/s, tuc sai so ~5.4% voi nhieu.
Thoi gian tinh: 94 us/lan (4.7% ngan sach 500Hz).

## HAI GIOI HAN PHAI BIET

1. **Goi duoi thang la diem ky di.** Jacobian chan o goi = 0 rad co
   `cond ~ 2e6`. Uoc luong luc tiep xuc sai RMS **99 N**; gap goi 0.3 rad con
   **1.4 N**. Luon dung goi chung.

2. **Uoc luong luc bo qua quan tinh `M*qddot`.** Chi dung o che do tua tinh.
   Luc bi day manh sai so len ~1600 N. Khi **di bo** phai dung cam bien luc
   ban chan hoac dua `qddot` vao. Van toc `v_body` KHONG bi anh huong -
   no la phep chieu thuan, khong nghich dao gi.

Phat hien tiep xuc moi kiem chung o pha **hai chan cham dat**. Chua kiem chung
gi cho pha mot chan.

## An toan

Node nay **chi doc**: subscribe `rt/lowstate`, publish `nav_msgs/Odometry`.
Khong publish `rt/lowcmd`, khong goi `LocoClient`, khong gui bat cu lenh nao
den robot. Chay song song voi bo dieu khien co san cua Unitree la an toan.

## Moi truong (da dung san tren may nay)

Phu thuoc **khong nam o python he thong**. Ubuntu 24.04 chan `pip install` vao
python he thong, va ROS Jazzy khong kem CycloneDDS (mac dinh la FastDDS), nen
moi thu nam trong mot venv ke thua site-packages:

    /home/dung/study/.venv-ros
      pinocchio 4.1.0        (pip "pin")
      unitree_sdk2py         (pip -e /home/dung/unitree_sdk2_python)
      cyclonedds 0.10.2      (binding python)
      libddsc 0.10.5         (thu vien C, build tu nguon vao chinh venv nay)

Package duoc build BANG python cua venv do, nen shebang cua node tro thang vao
day. **Khong can activate venv.**

    source G1/src/g1_leg_odometry/scripts/env.sh

Neu phai dung lai tu dau tren may khac:

    python3 -m venv --system-site-packages .venv-ros
    git clone --depth 1 -b releases/0.10.x \
        https://github.com/eclipse-cyclonedds/cyclonedds.git /tmp/cyclonedds
    cmake -S /tmp/cyclonedds -B /tmp/cyclonedds/build \
        -DCMAKE_INSTALL_PREFIX=$PWD/.venv-ros -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_TESTING=OFF -DBUILD_EXAMPLES=OFF
    cmake --build /tmp/cyclonedds/build -j$(nproc) --target install
    CYCLONEDDS_HOME=$PWD/.venv-ros .venv-ros/bin/pip install cyclonedds==0.10.2
    .venv-ros/bin/pip install pin -e /home/dung/unitree_sdk2_python
    cd G1 && ../.venv-ros/bin/python -m colcon build \
        --packages-select g1_description g1_leg_odometry

(Ban `sudo apt install ros-jazzy-cyclonedds ros-jazzy-pinocchio` cung duoc,
nhung phien ban CycloneDDS cua apt co the khong khop ban 0.10.2 ma
unitree_sdk2py ghim - build tu nguon thi chac chan khop.)

## Chay

    # co robot that
    ros2 launch g1_leg_odometry leg_odometry.launch.py network_interface:=eth0

    # khong co robot - kiem tra duong ong ROS
    ros2 run g1_leg_odometry leg_odometry_node --ros-args \
        -p enable_sdk:=false -p urdf_path:=<...>/g1_29dof.urdf

## Ghep vao stack

Sua **mot dong** trong `g1_state_estimator/config/estimator.yaml`:

    leg_topic: "/dog_odom"    ->    leg_topic: "/leg_odom"

Khong phai sua `estimator.cpp`.

## Kiem thu

    # doi chieu ground truth MuJoCo (can mujoco + quadprog)
    .venv-real/bin/python G1/src/g1_leg_odometry/test/test_against_mujoco.py [--noise]

    # duong ong ROS, khong can robot/DDS
    python3 G1/src/g1_leg_odometry/test/test_node_pipeline.py

---

# RUNBOOK: do tren robot that

Muc tieu buoi nay: lay con so `vel_std` / `leg_vel_noise` bang du lieu that,
va xem uoc luong co hop ly khong - TRUOC khi co bat ky mo-men nao do ta phat ra.

Ca hai node deu **chi doc**. Khong can tat bo dieu khien cua Unitree. Khong
can treo gian. Robot van o che do binh thuong cua no suot buoi.

## Chuan bi

    sudo apt install ros-jazzy-pinocchio
    cd <workspace>/G1 && colcon build --packages-select g1_description g1_leg_odometry
    source install/setup.bash
    ip a                      # xac dinh ten card mang noi toi robot

## Buoc 1 - robot dung yen

Cho robot vao che do dung cua Unitree:

    ros2 topic pub /g1_mode std_msgs/msg/String "data: 'stand'" --once

Roi chay:

    ros2 launch g1_leg_odometry check_on_robot.launch.py \
        network_interface:=eth0 csv_path:=/tmp/legodom.csv

Kiem tra ngay tren log:

| Thay gi | Nghia la |
|---|---|
| `/leg_odom ~500 Hz` | duong DDS thong |
| `0 Hz` + canh bao | sai `network_interface`, hoac robot khong phat lowstate |
| `fz L~160 R~160 N` | dung ca hai chan, tong ~ trong luong robot |
| `fz` lech nhau nhieu | robot dang doi chan, hoac uoc luong mo-men lech |
| `v_body ~ [0 0 0]` | dung nhu mong doi khi dung yen |

**De yen it nhat 60 giay.** Day la phan quan trong nhat: van toc that = 0 nen
do lech chuan do duoc chinh la nhieu phep do.

## Buoc 2 - co chuyen dong

Day nhe vao vai robot vai lan, roi cho di vai buoc:

    ros2 topic pub /g1_mode std_msgs/msg/String "data: 'walk'" --once

Xoay tai cho mot vong - buoc nay de kiem tra he quy chieu cua nguon doi chieu
(xem ghi chu trong bao cao).

## Buoc 3 - doc bao cao

Ctrl-C. Chep hai con so vao config:

    # g1_leg_odometry/config/leg_odometry.yaml
    vel_std: <so bao cao in ra>

    # g1_state_estimator/config/estimator.yaml
    leg_vel_noise: [<phuong sai>, <phuong sai>, <phuong sai>]
    leg_topic: "/leg_odom"        # doi tu /dog_odom

## Doc ket qua the nao

**Do lech chuan** (mong doi ~10-30 mm/s): con so chinh, dung lam `vel_std`.
Lon hon 50 mm/s thi co van de - xem CSV xem la nhieu trang hay dao dong co chu ky.

**Do lech he thong** (mong doi < 20 mm/s): neu lon hon nhieu, ba kha nang theo
thu tu de kiem tra truoc:
1. Gyro chua tru bias - de robot dung yen, xem `imu_state.gyroscope` co lech 0 khong.
2. Chan bi truot - gia dinh "chan tua khong truot" bi vi pham.
3. Nguong tiep xuc sai - xem `fz` trong log, chinh `contact_on`/`contact_off`.

**Doi chieu voi nguon doc lap**: neu lech lon **chi khi robot xoay**, do gan
nhu chac chan la khac he quy chieu (leg odometry tra ve he THAN), khong phai
loi uoc luong.

## Mot khac biet so voi mo phong - phai luong truoc

Trong mo phong minh dung mo-men **lenh**. Tren robot that, `motor_state.tau_est`
duoc suy ra tu dong dien dong co, nen co them ma sat hop so va sai so uoc luong.
Vi vay **uoc luong luc `fz` se te hon trong mo phong** va nguong tiep xuc
nhieu kha nang phai chinh lai.

Quan trong: dieu nay **khong anh huong toi van toc**. Cong thuc
`v_B = -(w x p + J qdot)` khong dung toi mo-men - `tau_est` chi vao phan phat
hien tiep xuc. Neu `fz` xau ma `v_body` van tot thi van dung duoc; chi can
chinh nguong, hoac tam thoi ep gia dinh hai chan cham dat khi robot dang dung.
