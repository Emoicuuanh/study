# WBC giữ thăng bằng cho Unitree G1 EDU — tiến trình

Ghi lại toàn bộ công việc từ câu hỏi ban đầu ("WBC tôi đang test là test gì")
đến trạng thái sẵn sàng chạy trên robot thật.

**Trạng thái:** bước 0 và bước 1 xong, đã kiểm chứng trong mô phỏng. Package ROS 2
build được, 4 test pass, môi trường dựng xong, đang chờ số đo từ robot thật.

---

## 1. Điểm xuất phát và một hiểu nhầm phải gỡ trước

Câu hỏi ban đầu là về [`week9_wbc.py`](week9_wbc.py) — WBC hai tay đuổi hai quả bóng
mocap, dùng QP có trọng số để phân xử hai tác vụ cạnh tranh. Mục tiêu là hai quả
bóng **chạy theo công thức cứng**, không có nhận diện gì; chúng là `mocap` body với
`contype=0 conaffinity=0`, tức không va chạm, chỉ để mắt người nhìn thấy đích.

Khi chuyển sang mục tiêu **giữ thăng bằng trên robot thật**, WBC tuần 9 **không dùng
lại được** — không phải vì thông số, mà vì sai loại bài toán:

> Tuần 9 giải theo **vận tốc khớp** `dq`, ngầm giả định "ra lệnh `dq` thì khớp đi đúng
> thế". Đúng với tay vẫy trong không khí. Sai hoàn toàn khi robot đứng trên chân, vì
> thứ quyết định ngã hay không là **lực tiếp xúc ở bàn chân** — đại lượng mà bài toán
> động học không hề có biến.

Chính docstring tuần 9 đã ghi lại điều này ở lần debug #3: thêm khớp eo vào để "giúp
giữ thăng bằng" → robot ngã sau 2–5s, kết luận *"không phải bug code, là giới hạn thật"*.
Kết luận đó vẫn đúng trên phần cứng, chỉ khác là lần này ngã thật.

Lộ trình đặt ra:

| Bước | Nội dung | Trạng thái |
|---|---|---|
| 0 | Leg odometry độc lập với `/dog_odom` | ✅ xong, đã kiểm chứng |
| 1 | WBC động lực học đầy đủ trong MuJoCo | ✅ xong, đã kiểm chứng |
| — | Port sang ROS 2, node chỉ đọc | ✅ xong, build + test pass |
| 2 | Chạy đo trên robot thật (không điều khiển) | ⏳ đang chờ |
| 3 | WBC lên phần cứng, treo giàn | chưa |

---

## 2. Bước 1 — WBC động lực học: [`week10_wbc_balance.py`](week10_wbc_balance.py)

### Bài toán

Ẩn số gồm **cả gia tốc khớp lẫn lực tiếp xúc**:

```
z = [ q̈ (nv=35) ; f (8 điểm × 3 = 24) ]      → 59 biến
```

Ràng buộc cứng (thứ tuần 9 hoàn toàn không có):

1. **Động lực học phần thân nổi** — 6 hàng đầu không có mô-men truyền động:
   `M[0:6,:] q̈ + h[0:6] = J_cᵀ[0:6,:] f`
   Đây chính là Newton–Euler toàn robot: chỉ lực tiếp xúc mới đổi được động lượng.
2. **Bàn chân không trượt** — ràng buộc **6D mỗi bàn chân**.
3. **Nón ma sát** tuyến tính hoá: `|fx|,|fy| ≤ μ fz`, `fz ≥ 0`. Tự động ép CoP nằm
   trong đa giác đỡ, vì 8 điểm chỉ đẩy được chứ không kéo được.
4. **Giới hạn mô-men**: `τ = M[6:,:] q̈ + h[6:] − J_cᵀ[6:,:] f ∈ [τ_min, τ_max]`.

Mục tiêu đặt dạng **centroidal**, thẳng lên lực — tránh phải tính `J̇_com`:

```
tịnh tiến:  Σ f_i            = m(ẍ_com_des − g)
quay:       Σ (p_i − c) × f_i = L̇_des = −Kp·θ_err − Kd·L
tư thế:     q̈[khớp] = Kp_q(q_ref − q) − Kd_q q̇
```

### Hai lỗi gặp trên đường

**`mj_fullM` đổi chữ ký ở MuJoCo 3.10** — giờ là `(m, d, dst)`, không còn nhận `data.qM`.

**"constraints are inconsistent".** Ban đầu đặt ràng buộc không trượt 3D cho *từng
điểm*: 4 điểm × 3 = 12 hàng mỗi bàn chân. Nhưng bàn chân là **vật rắn, chỉ 6 bậc tự do**
→ 12 hàng đó hạng 6, phụ thuộc tuyến tính, mà thuật toán Goldfarb–Idnani của `quadprog`
đòi các hàng đẳng thức độc lập. Bước đầu vẫn giải được vì `q̇=0` làm mọi thứ nhất quán,
sang bước hai là vỡ. Sửa: ràng buộc **6D theo từng bàn chân**, lực vẫn giữ ở 8 điểm.

### Kiểm chứng: đối chiếu lý thuyết capture point

Đây là phần đáng tin nhất, vì không dựa vào "nhìn thấy robot đứng được".

| Chiều đẩy | Biên bàn chân | F_crit lý thuyết (LIPM thuần) | Đo được |
|---|---|---|---|
| Lùi (về gót) | 5.3 cm | 44.6 N | **56–58 N** |
| Tới (về mũi) | 11.7 cm | 97.7 N | **90–120 N** |

Hai điều khớp: **bất đối xứng đúng theo hình bàn chân** (chịu đẩy về mũi ~gấp đôi về
gót, đúng tỉ lệ 11.7/5.3), và **vượt ngưỡng LIPM ~20–25%** — không phải sai số, mà vì
WBC còn điều tiết động lượng góc, thứ mà capture point thuần (điểm khối lượng, không
quán tính quay) không có.

Ngưỡng ngã ở 58 N **không phải bug**: quá capture point thì không mô-men cổ chân nào
cứu được, bắt buộc phải **bước chân** — chỗ [`week7_lipm_steps.py`](week7_lipm_steps.py)
nối vào sau này.

### Thời gian

```
WBC solve: trung bình 0.410 ms | p99 0.442 ms | max 0.635 ms
ngân sách 500Hz = 2.000 ms  →  dùng 20.5% trung bình, 31.8% xấu nhất
```

Python + NumPy + quadprog. Viết lại C++ với Pinocchio + ProxSuite sẽ còn nhanh hơn nhiều.

---

## 3. Bước 0 — Leg odometry: [`week11_leg_odometry.py`](week11_leg_odometry.py)

### Vấn đề

[`estimator.cpp:97`](G1/src/g1_state_estimator/src/estimator.cpp#L97) `UpdateLeg()` lấy
vận tốc thân từ `/dog_odom` ([`estimator.yaml:6`](G1/src/g1_state_estimator/config/estimator.yaml#L6)).
Topic đó do **SDK high-level sinh ra** — tức do chính bộ điều khiển mà WBC low-level sẽ
tắt. Khi chuyển sang `rt/lowcmd`, nguồn đó hoặc chết, hoặc tệ hơn là trả số cũ mà IEKF
vẫn tin. FAST-LIO thì chỉ 50–100 Hz, quá chậm cho vòng 500 Hz.

### Nguyên lý

Chân đang tựa (không trượt) ⇒ vận tốc chân = 0:

```
v_B = −(ω_B × p_BF + J_BF · q̇_leg)
```

**Không có ma trận quay ở vế phải.** Vận tốc thân *trong hệ thân* chỉ cần gyro và
encoder, không cần biết hướng robot — nên sai số hướng của IMU không lọt vào phép đo
này. Và đó đúng là đại lượng `UpdateLeg()` cần (`leg.v_body`).

FK/Jacobian tính trên bản sao mô hình với **thân ghim tại gốc, hướng đơn vị** — cố ý
không dùng pose thật của thân, vì robot thật không biết pose đó.

### Kết quả (đối chiếu ground truth MuJoCo, 12s có đẩy)

| | Sạch | Có nhiễu cảm biến |
|---|---|---|
| v_body RMS (3 trục) | 2.64 mm/s | **13.86 mm/s** |
| Trôi vị trí (lọc bù) | 0.2 mm/s | 1.9 mm/s |
| fz tĩnh, sai số RMS | 1.4 N | 1.5 N |
| Phát hiện tiếp xúc | 99.9% | 99.9% |

Biên độ vận tốc thật trong bài test: 257 mm/s → sai số ~5.4% với nhiễu. Với `KD_COM=15`
của WBC, 14 mm/s sai số chỉ đẻ ra ~7 N sai lệch lực, so với 327 N trọng lượng.

### Phát hiện quan trọng: keyframe chuẩn là điểm kỳ dị

Keyframe `stand` của mujoco_menagerie để **mọi khớp chân = 0 rad**, gối duỗi thẳng:

```
gối 0.00 rad (thẳng):  cond = 2.0e6   σ_min = 9.0e-7
gối 0.30 rad (17°):    cond = 47      σ_min = 0.039
```

Hậu quả đo được: sai số ước lượng lực tĩnh **99 N khi gối thẳng, 1.4 N khi gối chùng
0.3 rad**. Mọi thứ cần nghịch đảo Jacobian chân — ước lượng lực từ mô-men, IK,
admittance — đều vỡ ở tư thế đó. Đây là lý do mọi bộ điều khiển thăng bằng humanoid
thật đều đứng gối chùng.

Đã thêm `set_stand_pose()` và cờ `--bend` vào `week10`; WBC đứng vững ở 0.0/0.3/0.5 rad.
**Khi lên robot thật: đứng gối chùng.**

### Ghi chú API MuJoCo 3.10

- `mjENBL_SENSORNOISE` **đã bị bỏ** — thuộc tính `noise=` trong
  [`g1.xml:340-343`](mujoco_menagerie/unitree_g1/g1.xml#L340-L343) giờ chỉ là khai báo,
  không tự áp dụng. Phải đọc `model.sensor_noise` rồi tự cộng.
- `mj_step` không cập nhật lại `xpos` sau khi tích phân; muốn sai phân vị trí phải gọi
  `mj_kinematics`. Script có assert kiểm chứng `qvel[0:3]` đúng là vận tốc thân hệ thế
  giới (lệch 1e-14).

---

## 4. Package ROS 2: [`G1/src/g1_leg_odometry/`](G1/src/g1_leg_odometry/)

```
g1_leg_odometry/leg_odometry.py         toán thuần (Pinocchio), không ROS/DDS
g1_leg_odometry/leg_odometry_node.py    node ROS 2 + DDS rt/lowstate
g1_leg_odometry/leg_odom_check_node.py  công cụ đo trên robot thật
test/test_against_mujoco.py             đối chiếu ground truth MuJoCo
test/test_node_pipeline.py              đường ống ROS, không cần robot/DDS
test/test_check_node.py                 toán của công cụ đo, dữ liệu tổng hợp
config/, launch/, scripts/env.sh, README.md
```

### Kiểm chứng

| Test | Kết quả |
|---|---|
| Pinocchio vs MuJoCo, sạch | PASS — RMS **2.64 mm/s** |
| Pinocchio vs MuJoCo, có nhiễu | PASS — RMS **13.86 mm/s** |
| Đường ống ROS | PASS |
| Toán công cụ đo | PASS |

Hai con số đầu **trùng khít** bản MuJoCo thuần (2.64 / 13.83) → URDF `g1_29dof.urdf`
và mô hình MJCF khớp nhau tới độ chính xác số; nếu offset link hai bên lệch thì sai số
đã bung ra ngay. Thứ tự khớp `G1JointIndex` của SDK cũng trùng đúng thứ tự trong cả
URDF lẫn MJCF.

Một kiểm chứng độc lập nữa: bơm mô-men viết tay vào node, nó suy ra **163.8 N mỗi chân
= 327.6 N**, so với mg = 327.1 N.

Công cụ đo được kiểm bằng dữ liệu tổng hợp: bơm nhiễu σ = [15, 12, 4] mm/s và bias
[3, −2, 1] mm/s, nó trả về [14.88, 11.84, 4.00] và [3.10, −2.20, 0.89]. Quan trọng vì
con số nó in ra sẽ được chép thẳng vào config.

Thời gian tính: **94 µs/lần** (4.7% ngân sách 500 Hz).

### Ghép vào stack — đúng một dòng

```yaml
# g1_state_estimator/config/estimator.yaml:6
leg_topic: "/dog_odom"   →   leg_topic: "/leg_odom"
```

Không phải sửa `estimator.cpp` — nó chỉ đọc `twist.twist.linear`, đúng dạng node publish.

### An toàn

Cả hai node **chỉ đọc**. Subscribe `rt/lowstate` và `rt/lf/sportmodestate`, publish ROS
topic. Không publish `rt/lowcmd`, không gọi `LocoClient`. Chạy song song với bộ điều
khiển có sẵn của Unitree là an toàn — đó là lý do đây là thứ đầu tiên nên chạy trên
robot thật: kiểm chứng ước lượng bằng dữ liệu thật **trước khi** có bất kỳ mô-men nào
do ta phát ra.

---

## 5. Những khác biệt giữa tài liệu và thực tế máy

Bốn thứ phát hiện khi triển khai, đều đã xử lý:

**Máy chạy ROS 2 Jazzy / Ubuntu 24.04 / Python 3.12**, không phải Foxy / Ubuntu 20.04
như [`G1_WBC_FOOTSTEP_NAVIGATION_PLAN.md`](G1/docs/G1_WBC_FOOTSTEP_NAVIGATION_PLAN.md)
ghi. Các lệnh `apt install ros-foxy-*` trong plan không chạy được.

**Không có gì publish `/dog_odom` hay `/dog_imu_raw`.** Tìm khắp workspace lẫn
`/home/dung` — hai topic đó chỉ xuất hiện ở phía *tiêu thụ* (estimator, footstep planner)
và trong tài liệu. Bridge sinh ra chúng không tồn tại trên máy này. Nên phương án
"so `/leg_odom` với `/dog_odom`" không chạy được; thay bằng `rt/lf/sportmodestate` trong
SDK (trường `velocity[3]`, ước lượng của chính Unitree, lấy thẳng qua DDS, không cần
bridge). Phép đo chính (`vel_std`) chỉ cần `/leg_odom` nên không bị chặn.

**Thiếu `setup.cfg` thì `ros2 run` không tìm thấy node.** Với setuptools mới, console
script bị cài vào `install/<pkg>/bin/` thay vì `lib/<pkg>/`. Đã thêm; các package Python
khác trong repo đều đã có sẵn file này.

**Phụ thuộc không nằm ở python hệ thống.** Ubuntu 24.04 chặn `pip install` vào python hệ
thống, và ROS Jazzy không kèm CycloneDDS (mặc định FastDDS). Đã dựng venv
`.venv-ros` kế thừa site-packages:

```
pinocchio 4.1.0        (pip "pin")
unitree_sdk2py         (pip -e /home/dung/unitree_sdk2_python)
cyclonedds 0.10.2      (binding python)
libddsc 0.10.5         (thư viện C, build từ nguồn vào chính venv — khớp đúng
                        bản 0.10.2 mà unitree_sdk2py ghim, không cần sudo)
```

Package được build **bằng python của venv đó**, nên shebang node trỏ thẳng vào đây —
không cần activate. Chỉ cần `source G1/src/g1_leg_odometry/scripts/env.sh`.

Một bẫy đáng ghi: pinocchio cài bằng pip dùng layout `cmeel` nạp qua file `.pth`. Đặt
`PYTHONPATH` tới `site-packages` là **không đủ** — `.pth` chỉ được xử lý cho site
directory thật, nên phải dùng python của venv (hoặc thêm cả
`site-packages/cmeel.prefix/lib/python3.12/site-packages`).

---

## 6. Giới hạn đã biết

**Ước lượng lực chỉ dùng được ở chế độ tựa tĩnh.** Bỏ qua số hạng quán tính `M q̈`, nên
lúc bị đẩy sai số vọt lên ~1600 N. Khi **đi bộ** phải dùng cảm biến lực bàn chân hoặc
đưa `q̈` vào. Vận tốc `v_body` **không** bị ảnh hưởng — nó là phép chiếu thuận, không
nghịch đảo gì, và không đụng tới mô-men.

**Phát hiện tiếp xúc mới kiểm ở pha hai chân chạm đất.** Con số 99.9% là bài dễ; chưa
kiểm chứng gì cho pha một chân.

**Mô phỏng dùng mô-men lệnh; robot thật trả `tau_est` suy từ dòng điện động cơ**, có
thêm ma sát hộp số và sai số ước lượng. Nên `fz` trên phần cứng sẽ tệ hơn và ngưỡng tiếp
xúc nhiều khả năng phải chỉnh lại. Lại nhấn mạnh: điều này không ảnh hưởng tới vận tốc.

**Chưa xác nhận** G1 EDU của bạn có cảm biến lực bàn chân không, và tên topic
`rt/lf/sportmodestate` có đúng cho G1 không (phần lớn example trong SDK là cho Go2/H1).
Cũng chưa xác nhận hệ quy chiếu của trường `velocity[3]` là hệ thân hay hệ thế giới —
bước xoay tại chỗ trong quy trình đo là để phân biệt việc này.

---

## 7. Bước tiếp theo: đo trên robot thật

Xem runbook đầy đủ trong [`G1/src/g1_leg_odometry/README.md`](G1/src/g1_leg_odometry/README.md).
Tóm tắt:

```bash
# 1. cắm cáp ethernet vào enp2s0, đặt IP
sudo ip link set enp2s0 up
sudo ip addr add 192.168.123.222/24 dev enp2s0
ping -c3 192.168.123.161
ip route get 192.168.123.161        # phải thấy dev enp2s0

# 2. nạp môi trường
source /home/dung/study/G1/src/g1_leg_odometry/scripts/env.sh

# 3. chạy
ros2 launch g1_leg_odometry check_on_robot.launch.py \
    network_interface:=enp2s0 csv_path:=/tmp/legodom.csv
```

Ba thứ phải đúng trong 5 giây đầu: **~500 Hz**, **fz hai chân cộng lại ≈ 330 N**,
**v ≈ 0 khi robot đứng yên**.

Quy trình: đứng yên **≥60s** (ra con số `vel_std`) → đẩy nhẹ 20s → đi vài bước 20s →
xoay tại chỗ 20s (phân biệt lệch do khác hệ quy chiếu với lệch do sai ước lượng) →
Ctrl-C đọc báo cáo.

Báo cáo in thẳng hai dòng để chép vào `leg_odometry.yaml` và `estimator.yaml`
(lưu ý IEKF dùng **phương sai**, không phải độ lệch chuẩn).

Sau đó mới tới bước 2: port WBC sang C++/ROS 2 (`g1_wbc_balance_node`, Pinocchio +
ProxSuite) rồi lên phần cứng có treo giàn.

---

## 8. Buổi đo đầu tiên trên robot thật (2026-09-18)

Robot đứng bằng **bộ cân bằng của hãng**, hai node của mình chỉ đọc. Ethernet
`enp2s0`, DHCP 192.168.123.36, robot 192.168.123.161.

### Hạ tầng — những gì phải sửa tại chỗ

| Vấn đề | Nguyên nhân | Đã xử lý |
|---|---|---|
| `rt/lf/sportmodestate` không nhận được gì | robot publish kiểu `unitree_hg::SportModeState_`, SDK bản này **không có** IDL đó | đổi sang `rt/odommodestate` (kiểu `unitree_go`, 500 Hz) |
| Mất 46/72s dữ liệu giữa phép đo | cáp lỏng → link chớp (`carrier_changes=4`) → CycloneDDS mất multicast, **không tự khôi phục** | watchdog thoát + `respawn=True` trong launch |
| Báo cáo không in ra | Ctrl-C lần hai giết `final()` | khoá SIGINT/SIGTERM trước khi dựng báo cáo |
| `channel factory init error` khó hiểu | card mạng DOWN | kiểm tra card trước khi khởi tạo DDS, báo "KIEM TRA CAP ETHERNET" |

`rt/lowstate` chạy **1052 Hz**, không phải 500 Hz như giả định ban đầu.

Đính chính mục 5: `rt/dog_odom` và `rt/dog_imu_raw` **có tồn tại** — là DDS topic
từ robot, kiểu `nav_msgs::msg::dds_::Odometry_` và `sensor_msgs::msg::dds_::Imu_`.
Trước đó mình chỉ tìm trong workspace nên kết luận thiếu. Chúng là ROS-typed DDS,
nên nếu host dùng `rmw_cyclonedds_cpp` cùng domain thì sẽ thấy như ROS topic thật.

### Kết quả đo (26.1s, robot đứng, 27474 mẫu)

**Nhiễu cảm biến ≈ 6 mm/s.** Độ lệch chuẩn thô (62 mm/s ở trục x) **không phải
nhiễu** — bộ cân bằng của hãng làm robot dao động thật ở 1.63 Hz. Tách bằng hiệu
hai mẫu liên tiếp (dao động thật mượt ở thang 1ms, nhiễu trắng thì không):

```
x: tổng 62.37 = nhiễu 6.11 + dao động thật 62.07 mm/s
y: tổng 12.06 = nhiễu 4.59 + dao động thật 11.15 mm/s
z: tổng  5.04 = nhiễu 0.79 + dao động thật  4.97 mm/s
```

Đối chứng mạnh: lúc robot **treo** (chân không tải) đo được 5.79 / 5.76 / 0.77 mm/s
— gần như trùng khít. Nhiễu không đổi giữa hai điều kiện ⇒ đúng là nhiễu cảm biến,
không phải trượt chân.

```
vel_std       = 0.010
leg_vel_noise = [1.00e-04, 1.00e-04, 1.00e-04]
```

Phổ Welch: đỉnh 1.63 Hz cả ba trục, tỉ lệ mật độ 0.2-3Hz / >100Hz = 10621x (trục x).

### Kiểm chứng: đối chiếu ước lượng độc lập của Unitree

| Trục | Tương quan (<5Hz) | Hệ số tỉ lệ | Biên độ ta vs Unitree |
|---|---|---|---|
| x | **+0.904** | 0.95 | 53.9 vs 50.5 mm/s |
| y | +0.346 | 0.12 | 9.5 vs 10.9 mm/s |
| z | **+0.896** | 1.16 | 4.4 vs 3.2 mm/s |

Hai ước lượng **hoàn toàn độc lập** (khác thuật toán, khác cài đặt) khớp nhau ở
trục x và z cả về pha lẫn biên độ. Đây là kiểm chứng mạnh nhất có được khi không
có ground truth.

### Vấn đề còn mở: trục y

Đã **loại trừ** giả thuyết lệch hệ quy chiếu: quét góc xoay quanh z từ −30° đến
+30° cho thấy y tăng đều, không có đỉnh, còn x gần như không đổi — dấu hiệu của
rò tín hiệu chứ không phải căn chỉnh. Góc Procrustes khớp nhất (−3.65°) còn làm
y **tệ đi** (0.346 → 0.215).

Dấu vết thật nằm ở phân bố lực:

```
robot dao ngang 1.63 Hz, trọng lượng dịch giữa hai chân 75 N đỉnh-đỉnh
tương quan(chênh lực, v_y của Unitree) = -0.685
tương quan(chênh lực, v_y của ta)      = -0.060    <- không liên quan
tương quan(chênh lực, SAI LỆCH hai bên)= +0.590
```

Nhưng độ kết dính theo tần số lại cho thấy ở dải dao động chính (1.0–2.5 Hz)
coherence trục y đạt **0.736** — tức hai bên vẫn đồng thuận ở chỗ có tín hiệu.
Tương quan thô 0.346 bị pha loãng bởi các dải khác.

**Giả thuyết (chưa chứng minh):** chân đang nhẹ tải bập bênh trên mép, gốc cổ chân
không đứng yên, nhưng trung bình cộng vẫn coi nó ngang chân đang chịu tải.

Đã thêm tham số `fusion: "force" | "mean"` và **ghi vận tốc từng chân** vào
`/leg_contact` + CSV để buổi sau so trực tiếp. Lưu ý trong mô phỏng `mean` tốt hơn
(2.64 vs 2.95 mm/s) vì ở đó hai bàn chân phẳng hoàn hảo — chưa có cơ sở chọn bên nào.

### Ước lượng lực: cao hơn 15.8%

```
trái 188.1 ± 16.0 N | phải 190.7 ± 13.8 N | tổng 378.8 N
mg (URDF 33.34 kg) = 327.1 N  ->  lệch +15.8%
chênh lệch trái-phải: 2.5 N
```

Cân bằng trái-phải rất tốt (2.5 N). Phần dư ~51.7 N ≈ 5.3 kg nhiều khả năng là
**khối lượng thật không có trong URDF**: quét DDS thấy `rt/dex3/left/state` và
`rt/dex3/right/state` nên robot **có lắp tay Dex3**, chưa kể lidar. Phát hiện tiếp
xúc vẫn đúng, nhưng muốn dùng `fz` làm số tuyệt đối thì phải sửa khối lượng URDF
hoặc hiệu chuẩn hệ số.

### Chưa làm

Quy trình mới chạy phần **đứng yên**. Chưa đẩy, chưa đi bộ, chưa xoay tại chỗ —
nên hệ quy chiếu của `rt/odommodestate.velocity` vẫn chưa xác nhận dứt điểm, và
chưa có dữ liệu ở chế độ động.

---

## 9. WBC chế độ bóng trên robot thật (2026-09-18)

Package mới [`G1/src/g1_wbc/`](G1/src/g1_wbc/): WBC động lực học viết lại bằng
Pinocchio, chạy trên URDF + tải trọng thật, nhận trạng thái từ `rt/lowstate`,
**không gửi lệnh nào**. Robot vẫn do bộ cân bằng của hãng giữ.

### Kiểm chứng trong mô phỏng trước

| | Pinocchio/URDF | week10 (MuJoCo) | Lý thuyết capture point |
|---|---|---|---|
| Ngưỡng đẩy về gót | 62–70 N | 56–58 N | 44.6 N |
| Chịu sai khối lượng | 15.8% vẫn đứng | 15.8% vẫn đứng | — |
| Thời gian giải | 0.506 ms | 0.410 ms | — |

### Lỗi khái niệm: khung quy chiếu

Bản đầu đứng được 3.8s rồi đổ từ từ, **dù mọi đại lượng động lực học đều khớp
MuJoCo tới 1e-5** — `M`, `h`, Jacobian, số hạng trôi, động lượng góc, sai số
hướng, tất cả.

Nguyên nhân: ghim `q[:3] = 0` với lý lẽ "vị trí tuyệt đối không vào động lực
học". Đúng với `M` và `h` (trọng lực đều) nhưng **sai với tác vụ CoM**. Khi robot
nghiêng về trước quanh cổ chân, trong mô hình ghim gốc thì thân và CoM coi như
đứng yên còn bàn chân mới là thứ dịch chuyển — nên CoM so với gốc thân gần như
không đổi, tác vụ CoM không thấy sai lệch và không kéo lại.

Sửa bằng `_anchor()`: dịch mọi vị trí sao cho trung bình điểm tiếp xúc trùng với
lúc chốt mốc. Khung tham chiếu khi đó đứng yên cùng mặt đất.

Đáng ghi vì kiểm tra từng thành phần **không bắt được** loại lỗi này — mọi con số
đều đúng, chỉ ý nghĩa vật lý của khung quy chiếu là sai.

### Kết quả trên robot thật (33 s, 16539 chu kỳ)

**QP: 0 lần thất bại.** Giải được ở mọi tư thế robot đi qua. Tổng lực pháp tuyến
QP giải ra 378.6 N, khớp đúng `mg` mô hình 378.7 N.

**Thời gian: không đạt.**

```
TB 0.988 ms | p99 4.04 ms | max 21.8 ms
>2ms: 9.42% chu kỳ | >5ms: 0.34% | >10ms: 0.05%
```

Các đỉnh không đều (khoảng cách trung vị 29 chu kỳ) → tranh chấp CPU/DDS, không
phải GC định kỳ. Python ở 500 Hz không giữ được hạn chót cứng. Đây là **bằng
chứng** cho việc phải port C++, không còn là phỏng đoán.

**Mô-men: cùng bậc độ lớn, phân bố khác hẳn.** `|tau|max` ta 10.8–13.6 vs Unitree
14.7–14.9 Nm. Tương quan từng khớp gần 0 — điều này *không* đáng báo động vì đứng
hai chân là bài toán siêu tĩnh định, không gian rỗng rất lớn. Nhưng tổng mô-men
chân của ta thấp hơn gần gấp đôi (17.2/21.0 so với 29.0/39.4 Nm) — **chưa giải
thích được, đang điều tra**.

### Rung mô-men: giả thuyết sai, đo mới ra

Robot đứng yên mà mô-men của ta dao động **1.217 Nm** trong khi Unitree chỉ
**0.076 Nm** — gấp 16 lần. Áp lên robot thì đó là rung xóc.

Giả thuyết ban đầu (`KD_Q` × nhiễu encoder) **sai**. Quét tắt từng số hạng:

| | rung (Nm) |
|---|---|
| gốc | 1.257 |
| `kd_q = 0` | 1.277 — không liên quan |
| `kd_ang = 0` | 0.870 |
| `kd_com = 0` | **0.467** — thủ phạm |

Cơ chế: nhiễu vận tốc thân 6 mm/s × `kd_com` × khối lượng → ~3.5 N nhiễu lực →
qua cánh tay đòn tiếp xúc ~0.3 m → ~1 Nm. Mô phỏng dùng vận tốc hoàn hảo nên
không bao giờ lộ.

### Chọn lọc thay vì hạ hệ số

| | Ngưỡng đẩy (sim, tính cả trễ lọc) | Rung (robot thật) |
|---|---|---|
| gốc, không lọc | 62–70 N | 1.289 Nm |
| **gốc + lọc v10/dq10** | **62–70 N** | **0.57–0.86 Nm** |
| hạ hệ số (8/20) + lọc | 55–62 N | 0.304 Nm |

Lọc 10 Hz **miễn phí**: dao động thăng bằng chỉ 1.6 Hz nên trễ pha không đáng kể.
Đã đặt mặc định trong [`g1_wbc/config/wbc.yaml`](G1/src/g1_wbc/config/wbc.yaml).

Hai điều cần nhớ:
- **Lọc 5 Hz tệ hơn 10 Hz** (0.414 vs 0.304 Nm) — trễ pha quá nhiều tự sinh dao động.
- Con số rung **dao động giữa các lần chạy** (0.568 → 0.71–0.86), nên mức cải
  thiện thật là 1.5–2.3 lần, chưa chốt được một con số.

Muốn giảm nữa thì **đừng detune tiếp** — rung còn lại bị chặn dưới bởi chất lượng
ước lượng vận tốc. Đường đúng là hợp nhất IMU bằng IEKF để hạ nhiễu 6 mm/s.

### Xung đột RMW do chính mình gây ra

Việc bật `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` ở mục 8 **làm hỏng mọi node dùng
SDK**. Robot phát trên CycloneDDS domain 0; khi ROS cũng dùng CycloneDDS thì trong
một tiến trình có hai bên cùng đòi tạo domain 0 với hai cấu hình khác nhau:

```
ros trước : [ChannelFactory] create domain error
sdk trước : rmw_create_node: failed to create domain, Precondition Not Met
```

Thử cả hai thứ tự đều hỏng. Đã gỡ khỏi mặc định: `source env.sh` để RMW mặc định
(cho node SDK), `source env.sh ros_bridge` mới bật CycloneDDS cho tiến trình riêng
**không dùng SDK**.

Hướng đúng lâu dài: sinh gói ROS message cho kiểu `unitree_hg` như `unitree_ros2`
chính thức làm, rồi subscribe `rt/lowstate` như ROS topic — bỏ SDK khỏi tiến trình
thì hết xung đột.

### Còn lại

1. **Tổng mô-men thấp hơn Unitree gần gấp đôi** — chưa giải thích. Phải loại trừ
   khả năng WBC bù thiếu trước khi cho nó cầm lái.
2. **Port C++** (Pinocchio + ProxSuite) — đã có bằng chứng về thời gian.
3. Hợp nhất IMU để hạ nhiễu vận tốc, giảm rung mà không detune.

---

## 10. Bộ ghi dữ liệu thô và test vector (2026-09-29)

Mục này giải quyết hai lỗi của buổi đo trước, cùng lúc:

1. **Ghi vào `/tmp`** → mất sạch sau khi khởi động lại máy.
2. **Chỉ ghi đầu ra đã xử lý** (`tau`, `v_body`, `fz`), không ghi đầu vào
   (`q`, `dq`, `quat`, `gyro`). Nên kể cả giữ được file cũng không phát lại được.

### Những gì đã thêm

| File | Việc |
|---|---|
| [`g1_wbc/recording.py`](G1/src/g1_wbc/g1_wbc/recording.py) | schema `.npz` dùng chung cho bộ ghi và bộ phát lại |
| [`g1_wbc/lowstate_recorder_node.py`](G1/src/g1_wbc/g1_wbc/lowstate_recorder_node.py) | node **chỉ đọc** `rt/lowstate`, ghi thô vào `G1/data/recordings/` |
| [`g1_wbc/replay.py`](G1/src/g1_wbc/g1_wbc/replay.py) | chạy lại leg odometry + WBC trên bản ghi, **không cần robot** |
| [`g1_wbc/test_vectors.py`](G1/src/g1_wbc/g1_wbc/test_vectors.py) | `make` / `rerun` / `check` — sinh và đối chiếu file chuẩn cho bản C++ |
| [`test/test_record_replay_pipeline.py`](G1/src/g1_wbc/test/test_record_replay_pipeline.py) | kiểm chứng cả dây chuyền bằng bản ghi MuJoCo giả lập |
| [`launch/record.launch.py`](G1/src/g1_wbc/launch/record.launch.py) | `ros2 launch g1_wbc record.launch.py label:="..." secs:=40` |

Ghi **cả 35 động cơ** dù G1 chỉ dùng 29: ghi thô là ghi thô, cắt bớt là một quyết
định diễn giải, để dành ra lúc đọc. Bản ghi nằm trong repo nhưng **không vào git**
(`G1/data/recordings/.gitignore`) — 40 s ≈ 10 MB, git không hợp để giữ. File test
vector thì **có vào git**: đó là mốc so sánh, mất là phải có robot mới tạo lại được.

### Định dạng test vector

`.txt` phẳng, đọc bằng `ifstream >>` là xong — cố ý không dùng JSON để bản C++
không phải kéo thêm thư viện. Ngoài `IN_*`/`OUT_*` của từng vector, file còn chốt
cả **đầu vào của `capture_reference`** và kết quả của nó (`REF_QREF`, `REF_COMDES`,
`REF_ANCHOR`), để bản C++ đối chiếu được cả bước chốt mốc chứ không chỉ `solve()`.

Chọn khung bằng *farthest point sampling* trên `[q_chân, quat, v_body, tiếp xúc]`,
cộng thêm vài khung ở **giai đoạn một chân**. Lấy 40 khung liên tiếp thì được 40
bản sao gần giống nhau — vô dụng, vì bản C++ chỉ cần đúng ở một tư thế là đúng cả
chùm. Cần các tư thế **khác nhau**.

### Kiểm chứng khi chưa có robot

`test_record_replay_pipeline.py` dựng bản ghi giả lập từ MuJoCo **đúng định dạng
bộ ghi sẽ tạo**, có thêm nhiễu cảm biến theo mức đo được trên máy thật
(`dq` 12 mrad/s, `gyro` 2 mrad/s, `tau_est` 0.30 Nm), rồi cho đi hết dây chuyền.
Bốn bước, bước nào hỏng báo bước đó. Kết quả (6 s dữ liệu, 500 Hz):

```
1/4  3000 khung | 6.0 s | 500 Hz | 1.11 MB | gián đoạn 0
2/4  giải được 1249 chu kỳ, fail 0 | TB 0.560 ms | p99 0.909 | max 1.080
3/4  30 vector (rerun bản Python trên chính đầu vào đó) -> lệch 0.000e+00
4/4  tiêm lỗi -> bộ đối chiếu BẮT ĐƯỢC
PASS
```

Bước 4 quan trọng nhất và dễ bị bỏ qua nhất: **bộ đối chiếu không bắt được lỗi thì
nó vô dụng, và điều đó phải được kiểm tra chứ không được tin**. Nên test tiêm đúng
cái lỗi vẫn lấy làm ví dụ — đảo dấu `pin.skew(pts[i] - com)` — bằng cách thay
`pin` trong `g1_wbc.wbc` bằng một proxy trả `-skew`. Lỗi này **không crash, không
NaN, QP vẫn giải ra mô-men bậc 10–15 Nm**; chỉ dấu mô-men góc là ngược:

```
OUT_TAU: lệch lớn nhất 5.482e+01  *** LỆCH ***
    khớp  0- 5 (chân trái): 5.208e+01  LỆCH
    khớp 12-14 (eo       ): 5.482e+01  LỆCH
```

Và một lỗi thứ hai, chỉ sai ở ba khớp eo, để xem báo cáo có khoanh đúng vùng không:

```
OUT_TAU: lệch lớn nhất 8.400e+00  *** LỆCH ***
    khớp  0- 5 (chân trái): 0.000e+00  OK
    khớp 12-14 (eo       ): 8.400e+00  LỆCH
    khớp 15-21 (tay trái ): 0.000e+00  OK
  5 khớp lệch nhất: 14(8.40e+00), 13(8.40e+00), 12(8.40e+00), ...
```

Đó chính là thứ cần: không phải "có gì đó sai", mà "sai ở ba khớp này".

Ngưỡng `1e-6` Nm. Cùng công thức, cùng `double`, khác thứ tự phép cộng thì lệch cỡ
`1e-12`; mọi lỗi dịch thật (sai dấu, sai chỉ số, sai đơn vị) đều lệch ít nhất hàng
phần mười Nm. Không có vùng xám ở giữa.

### Giới hạn của phát lại

Phát lại **không có phản hồi**: robot trong bản ghi làm gì là làm rồi, mô-men ta
tính ra không làm nó đổi ý. Nên phát lại trả lời được "QP có giải được ở mọi tư thế
thật không", "đổi hệ số thì rung đổi bao nhiêu", nhưng **không** trả lời được "có
giữ được thăng bằng không" — chỗ đó vẫn phải MuJoCo (có phản hồi) hoặc robot thật.

### Còn phải làm khi có robot

Ghi 40 s có nhiễu động thật (đẩy vai, đi vài bước, xoay tại chỗ) rồi sinh lại test
vector từ bản ghi **thật** thay cho bản MuJoCo. Tư thế tự nghĩ ra chỉ phủ được các
trường hợp mình nghĩ ra; bản ghi thật chứa những thứ mình không nghĩ ra — và đó mới
là chỗ nhánh rẽ ít được đi qua nhất, tức là chỗ lỗi dịch nằm lâu nhất.

---

## 11. Bản C++ của WBC (2026-09-29)

[`G1/src/g1_wbc_cpp/`](G1/src/g1_wbc_cpp/) — Pinocchio 4.1 + ProxSuite 0.7.3 + yaml-cpp.

| File | Việc |
|---|---|
| [`include/g1_wbc/balance_wbc.hpp`](G1/src/g1_wbc_cpp/include/g1_wbc/balance_wbc.hpp) | giao diện, hằng số điểm tiếp xúc, `Gains`, `SolverOpts` |
| [`src/balance_wbc.cpp`](G1/src/g1_wbc_cpp/src/balance_wbc.cpp) | bản dịch của [`wbc.py`](G1/src/g1_wbc/g1_wbc/wbc.py) + `payload.py` |
| [`src/run_test_vectors.cpp`](G1/src/g1_wbc_cpp/src/run_test_vectors.cpp) | đối ứng C++ của `test_vectors.rerun()` |
| [`test/cross_check.sh`](G1/src/g1_wbc_cpp/test/cross_check.sh) | một lệnh: chạy C++ rồi đối chiếu với bản Python |

Dịch **từng bước một**, cố ý giữ nguyên thứ tự gọi hàm Pinocchio và thứ tự cộng
ma trận của bản Python. Các hàm đó có tác dụng phụ lên `data`, đổi thứ tự là đổi
kết quả; và khi lệch thì muốn lệch do **lỗi dịch** chứ không phải do "mình làm
theo cách khác". Tối ưu tốc độ để sau — lúc đó đã có test vector giữ lưng.

### Kết quả đối chiếu

```
mo hinh C++: 38.600142 kg | chuan: 38.600142 kg
  MASS         lệch 7.105e-15  OK
  REF_COMDES   lệch 0.000e+00  OK
  REF_ANCHOR   lệch 0.000e+00  OK
OUT_TAU:  lệch lớn nhất 2.967e-09  OK
OUT_QACC: lệch lớn nhất 5.959e-11  OK
OUT_F:    lệch lớn nhất 1.044e-08  OK
KHỚP - bản dịch đúng.
```

`2.967e-09` Nm chứ không phải `1e-15`: bản Python dùng `quadprog` (Goldfarb-Idnani,
phương pháp tập tích cực, nghiệm **chính xác**), bản C++ dùng ProxSuite (ADMM,
nghiệm **xấp xỉ đến ngưỡng**). Đặt `eps_abs = 1e-11` thì lệch còn `3e-9`, tức là
thấp hơn ngưỡng `1e-6` khoảng 300 lần. Phần lệch này là **sai số giải**, không
phải sai số dịch — và đó là lý do ngưỡng để `1e-6` chứ không phải `1e-12`.
ProxSuite còn giữ trạng thái nội bộ (tiền điều kiện, `rho`) giữa các lần gọi nên
kết quả xê dịch cỡ `1e-9` tuỳ lịch sử gọi; vẫn nằm sâu dưới ngưỡng.

### Hai chỗ cố ý làm khác bản Python

1. **Số hàng bất đẳng thức cố định.** Bản Python thêm/bớt hàng tuỳ chân có chạm
   đất không. Bản C++ giữ nguyên số hàng, chân bay thì đổi **cận** của hàng `fz`
   thành `[0, 1e-6]`. Tương đương về toán học, nhưng kích thước QP không đổi nên
   không phải cấp phát lại mỗi chu kỳ.
2. **Giới hạn mô-men gộp thành ràng buộc hai phía.** ProxSuite nhận `l ≤ Cz ≤ u`
   nên 58 hàng một phía gộp còn 29. Nghiệm không đổi.

Cả hai đều là thay đổi **có thể sai**, và cả hai đều được test vector xác nhận là
không sai — đúng công dụng của nó.

### Thời gian giải (30 vector, laptop, `--bench 20`)

| cấu hình | TB | p99 | max | vòng lặp QP |
|---|---|---|---|---|
| `eps 1e-11` (đối chiếu) | 0.52 ms | 0.68 | **0.72** | 19 |
| `eps 1e-7` | 0.47 ms | 0.64 | **0.68** | 16 |
| `eps 1e-7 --warm` | 0.45 ms | 1.91 | **2.41** | 26 (max 259) |
| `eps 1e-5 --warm` | 0.37 ms | 1.89 | **4.59** | 9 (max 117) |

Bản Python trên cùng dữ liệu mô phỏng: TB 0.56 ms, max 1.08 ms; **trên robot thật
p99 3.39 ms, xấu nhất 13.58 ms**. Khoảng cách nằm ở cái đuôi chứ không ở trung
bình — đúng thứ bộ thu gom rác gây ra, và đúng thứ giết một hạn chót 2 ms.

**Khởi động nóng làm xấu đi**, ngược với trông đợi: xuất phát từ nghiệm của một
tư thế **xa** thì ADMM mất nhiều vòng lặp hơn là xuất phát từ số không (259 vòng
so với 26). Các vector được chọn có chủ ý là cách xa nhau, còn hai chu kỳ liên
tiếp trên robot thì gần nhau — nên **chưa kết luận được** khởi động nóng tốt hay
xấu khi chạy thật, phải đo bằng dữ liệu liên tiếp thật. Trước mắt dùng khởi động
nguội: đã đủ nhanh (dư 2.8 lần) và cái đuôi **đoán trước được**, mà với hạn chót
cứng thì đuôi mới là thứ đáng giá.

### Chạy lại

```bash
cd G1 && colcon build --packages-select g1_wbc_cpp \
    --cmake-args -DPython3_EXECUTABLE=/usr/bin/python3
bash G1/src/g1_wbc_cpp/test/cross_check.sh
```

### Còn lại

1. File chuẩn hiện sinh từ **MuJoCo**, chưa phải robot thật. Ghi 40 s có nhiễu
   động thật rồi sinh lại — tư thế mô phỏng chỉ phủ những gì mình nghĩ ra.
2. Chưa có node ROS thời gian thực bản C++ (mới có chương trình đối chiếu).
   Bước sau: node chế độ bóng bản C++, đo p99 **trên robot thật**.
3. Chưa đo trên máy tính của robot, mới đo trên laptop.

---

## 12. Buổi đo thứ hai trên robot thật (2026-10-03)

Mạng: robot `192.168.123.161`, laptop `192.168.123.36` trên `wlp0s20f3`.

### Một lỗi của bộ ghi, lộ ra ngay lần chạy đầu

`default_dir()` tính đường dẫn bằng "lên bốn cấp từ `__file__`". Chạy qua
`ros2 run` thì `__file__` nằm trong `G1/install/g1_wbc/lib/python3.12/
site-packages/`, lên bốn cấp ra `G1/install/g1_wbc/` — bản ghi sẽ rơi vào cây
`install` và **mất sạch ở lần `colcon build` sau**. Đúng cái lỗi `/tmp` cũ,
mặc áo khác. Sửa: tìm ngược lên từ cả `__file__` lẫn thư mục hiện tại cho đến
khi thấy `G1/src/g1_wbc`; thêm biến `G1_DATA_DIR` để ghi đè.

Cũng sửa: khi không có dữ liệu thì `t0` mãi là `None` nên `stop_after_secs`
không bao giờ chạm tới — node treo vô hạn. Giờ đếm từ lúc khởi động node.

### Chất lượng đường truyền wifi

```
chu ky den (ms): trung vi 0.84 | p99 6.34 | max 21.7   -> 1415 lan "gian doan"
tick cua robot : nhay >1 chi 4 lan | mat ~29/8252 goi = 0.35%
```

Hai con số nói hai chuyện khác nhau. **1415 "gián đoạn" là jitter wifi**: gói về
thành cụm rồi nghỉ, chứ không mất. Dấu hiệu chắc chắn là `tick` của chính robot —
nó chỉ nhảy 4 lần. Còn 415 cặp trùng `tick` là do robot phát ~1050 Hz, nhanh hơn
độ phân giải 1 ms của `tick`; kiểm tra nội dung thì 414/415 cặp **khác nhau**,
chỉ 1 cặp là gói lặp thật. Dữ liệu sạch.

Bài học: **đừng đo mất gói bằng đồng hồ của máy nhận.** Đồng hồ của nguồn phát
mới phân biệt được "mất" với "về muộn".

### Rung mô-men 3.198 Nm — và vì sao đó KHÔNG phải vấn đề

Phát lại bản ghi 42 s đứng yên: QP giải 21547 chu kỳ, **hỏng 0 lần**, TB 0.487 ms.
Nhưng rung mô-men chân của ta 3.198 Nm, Unitree 1.373 Nm — trong khi tháng 9 ta
chỉ 0.57–0.86. Tách nhiễu khỏi chuyển động bằng sai phân bậc 1:

| | x | y | z |
|---|---|---|---|
| `v_body` độ lệch chuẩn | 35.79 | 7.00 | 1.45 mm/s |
| phần **nhiễu trắng** | 7.02 | 5.62 | 0.80 mm/s |
| phần **chuyển động thật** | 35.10 | 4.18 | 1.21 mm/s |

Nền nhiễu **không đổi** so với tháng 9 (6.11/4.59/0.79). Phổ của trục x: 75.9%
năng lượng ở dải 1–5 Hz — đúng dải dao động thăng bằng, không phải nhiễu trắng.
Hôm nay robot đung đưa trước-sau thật sự nhiều hơn, và bộ điều khiển của Unitree
cũng phản ánh (1.373 so với 0.076 Nm hồi tháng 9).

Tỉ lệ rung ta/Unitree: **hôm nay 2.3 lần, tháng 9 là 17 lần**. Nghĩa là khi tín
hiệu chủ yếu là chuyển động thật chứ không phải nhiễu, bản WBC bám sát hơn nhiều.
Nó không khuếch đại nhiễu.

Còn mở: hai chân ước lượng vận tốc lệch nhau **11.4 mm/s một cách hệ thống** trên
trục x (độ lệch chuẩn 43.6). Lớn hơn trước. Chưa rõ là trượt chân hay sai mô hình.

### Test vector từ dữ liệu THẬT

```
.venv-real/bin/python G1/src/g1_wbc/g1_wbc/test_vectors.py make \
    G1/data/recordings/lowstate_real_quiet.npz -o G1/data/test_vectors/real_quiet -n 40
bash G1/src/g1_wbc_cpp/test/cross_check.sh G1/data/test_vectors/real_quiet.txt
```

```
da giai 40 vector, that bai 0 | vong lap QP: TB 15 max 32
thoi gian giai: TB 0.3846 ms | p99 0.6742 | max 0.6792
OUT_TAU:  lech lon nhat 3.854e-09  OK
OUT_QACC: lech lon nhat 1.172e-11  OK
OUT_F:    lech lon nhat 1.038e-08  OK
KHOP - ban dich dung.
```

Bản C++ đã được kiểm chứng trên **tư thế robot thật**, không còn chỉ là MuJoCo.

### Còn lại

1. Bản ghi mới chỉ có robot **đứng yên** — 40/40 vector đều hai chân chạm đất,
   không có giai đoạn một chân. Cần bản ghi có nhiễu động thật.
2. Chưa có node thời gian thực bản C++.
3. Chưa đo trên máy tính của robot, mới đo trên laptop.

### Bản ghi có nhiễu động — và chỗ lệch mà nó lôi ra

Thêm **tự tìm card mạng**: máy này có hai card wifi và chúng **đổi chỗ mạng cho
nhau** giữa các lần kết nối. Ghi cứng tên card vào lệnh thì hỏng im lặng — node
chạy, không báo lỗi, chỉ là không bao giờ nhận được gói nào. Đã mất một phiên ghi
70 giây vì vậy. Giờ `network_interface: auto` tìm card đang ở `192.168.123.x`;
chỉ định tay mà sai thì cảnh báo chứ không chặn.

Bản ghi 66 s (`lowstate_real_disturb.npz`), đã dò lại từ dữ liệu xem thật sự xảy
ra gì chứ không ghi mốc theo ý định:

```
tiep xuc: hai chan 58.7% | MOT CHAN 41.3%
 0-10s   1 chan  0.0% | |v| TB  24 mm/s        <- de yen
10-30s   1 chan  7-17% | |v| max 921 mm/s      <- day vai
30-35s   1 chan  0.0% | |v| TB  13 mm/s        <- yen lai
35-65s   1 chan 73-87% | |v| max 1788 | yaw doi toi 95 do/5s   <- di va xoay
so lan vao giai doan MOT CHAN: 154 | dai nhat 0.55s | tong 27.2s
```

Phát lại: **34184 chu kỳ, QP hỏng 0 lần**, kể cả lúc đi và xoay. 55 vector, trong
đó **41 ở giai đoạn một chân** — đúng nhánh mà bản MuJoCo không phủ được.

Và nó lôi ra ngay một chỗ lệch: `OUT_TAU` 1.717e-06, vượt ngưỡng `1e-6` cũ. Lệch
**trải đều trên mọi nhóm khớp**, không khoanh vào nhóm nào — không giống dấu vân
tay của lỗi dịch. Siết `eps_abs`:

| eps_abs | lệch `OUT_TAU` |
|---|---|
| 1e-7 | 1.923e-06 |
| 1e-9 | 1.717e-06 |
| 1e-11 | **1.717e-06** (chững lại) |
| 1e-13 | 88 Nm (chạm `max_iter`, ghi số 0) |

Chững lại thì không phải sai số giải thuần tuý. Nên lấy chính bài toán QP ra kiểm
tra cả hai nghiệm (thêm cờ `keep_qp` vào `wbc.py`):

| | hai chân (14 vector) | một chân (41 vector) |
|---|---|---|
| lệch `tau` | 1.27e-09 | 1.72e-06 |

Trên ba vector lệch nhất: cả hai nghiệm đều **khả thi** (vi phạm ràng buộc ~1e-12)
và hàm mục tiêu chênh 2.4e-03 trên giá trị **6.7e+06** — khác nhau ở chữ số có
nghĩa thứ mười.

Nguyên nhân là **thang của bài toán**: `w_com=60` nhân với lực cỡ 400 N bình
phương cho hàm mục tiêu cỡ 1e7, nên sai số tương đối của `double` (1e-16) đã là
1e-9 tuyệt đối trên hàm mục tiêu; ở các hướng gần bằng phẳng trong giai đoạn một
chân, nó nở ra thành 1e-6 trên biến. **Không thể làm tốt hơn bằng `double`** —
đúng như bảng siết `eps_abs` cho thấy.

Nên ngưỡng đổi thành `1e-4` Nm, có lý do chứ không phải nới cho test chạy qua:
nhỏ hơn độ phân giải mô-men của động cơ nhiều bậc, mà vẫn cao hơn **1e5 lần** so
với hai lỗi tiêm có ý (8.4 và 54.8 Nm) — đã chạy lại và cả hai vẫn bị bắt, vẫn
khoanh đúng vào nhóm khớp eo. `eps_abs` mặc định hạ về `1e-9` vì siết hơn không
giúp gì mà tốn thêm vòng lặp.

Kết quả cuối, cả ba bộ:

| bộ vector | lệch `OUT_TAU` | giải TB | max |
|---|---|---|---|
| `real_disturb` (55, 41 một chân) | 1.717e-06 | 0.63 ms | 1.16 ms |
| `real_quiet` (40) | 7.018e-08 | 0.37 ms | 0.64 ms |
| `sim_pipeline` (30) | 3.385e-07 | 0.48 ms | 0.71 ms |

**Đáng ghi lại ngoài lề:** thang 1e7 của hàm mục tiêu là một điểm yếu thật của
công thức, không chỉ của bản C++. Chuẩn hoá lại trọng số (chia lực cho `m*g`,
chia gia tốc cho `g`) sẽ cải thiện điều kiện số cho **cả hai** bản. Chưa làm vì
sẽ phải chỉnh lại toàn bộ hệ số đã dò.

---

## 13. Node chế độ bóng bản C++ (2026-10-03, buổi chiều)

SDK C++ của Unitree không có trên máy → lấy về `/home/dung/unitree_sdk2` (có sẵn
`libunitree_sdk2.a` dựng sẵn cho x86_64). CMake để **tuỳ chọn**: không thấy SDK
thì vẫn build thư viện + chương trình đối chiếu, chỉ bỏ node sống.

| File | Việc |
|---|---|
| [`include/g1_wbc/leg_odometry.hpp`](G1/src/g1_wbc_cpp/include/g1_wbc/leg_odometry.hpp) · [`src/leg_odometry.cpp`](G1/src/g1_wbc_cpp/src/leg_odometry.cpp) | bản C++ của `leg_odometry.py` |
| [`src/shadow_node.cpp`](G1/src/g1_wbc_cpp/src/shadow_node.cpp) | node bóng, **chỉ một `ChannelSubscriber`** trên `rt/lowstate` |
| [`test/run_shadow.sh`](G1/src/g1_wbc_cpp/test/run_shadow.sh) | tự dò card mạng rồi chạy |

An toàn: không có `ChannelPublisher` nào, không `LocoClient`, không `rt/lowcmd`.

### Bộ kiểm chứng bắt được lỗi của chính nó

Thêm `IN_TAUEST` / `OUT_VFOOT` / `OUT_FZ` vào định dạng test vector để leg
odometry bản C++ cũng được đối chiếu. Kết quả đầu tiên:

```
OUT_VFOOT: lech lon nhat 9.850e-01   *** LECH ***
OUT_FZ:    lech lon nhat 6.821e-13   OK
```

Hai đại lượng này khác nhau đúng một chỗ: `v_foot` dùng `dq`, `fz` thì không.
Hoá ra định dạng chỉ lưu **một** `IN_DQ` là bản **đã lọc** (cái WBC nhận), trong
khi leg odometry bên Python chạy trên `dq` **thô**. Bản C++ bị cho ăn nhầm đầu
vào. Node sống vốn viết đúng — nhưng **bộ kiểm chứng không chứng minh được điều
đó**, và đó mới là vấn đề.

Thêm `IN_DQRAW` (định dạng 3). Cũng sửa: `--tol` mặc định là `TOL` nên đè lên
ngưỡng riêng từng đại lượng, `OUT_VFOOT` đang bị chấm bằng ngưỡng 1e-4 của QP
thay vì 1e-9. Giờ mặc định `None`.

| | `real_disturb` (41/55 một chân) | `real_quiet` (50) |
|---|---|---|
| `OUT_TAU` | 1.717e-06 | 7.018e-08 |
| `OUT_VFOOT` | **5.551e-17** | 1.735e-18 |
| `OUT_FZ` | 6.821e-13 | 1.705e-13 |

### Thời gian thật, chạy sống trên robot

~31000 chu kỳ liên tục ở 525 Hz (chia tần số 1/2), robot đứng cân bằng:

```
chu ky 31026 | QP fail 0 | odom bo 0
  ca chu ky: TB 0.86 | p99 1.45 | p999 1.68 | max 2.08 ms   (ngan sach 2.0 ms)
  rieng QP : TB 0.83 | p99 1.40 | max 2.05 ms
  rung mo-men chan: ta 3.32 vs Unitree 1.29 Nm | cung dau 80% | CoM lech max 8.2 mm
```

So với bản Python **trên cùng robot**: p99 3.39 ms, xấu nhất 13.58 ms. Bản C++
p99 1.45 ms, xấu nhất 2.08 ms — **cái đuôi co lại 6.5 lần**, đúng chỗ cần.

Nhưng vẫn chưa an toàn: max 2.08 ms đã **chạm mép** ngân sách 2.0 ms. Và QP
chiếm 0.83/0.86 ms, tức gần như toàn bộ — muốn cắt thì phải cắt ở QP (chuẩn hoá
thang bài toán, hoặc đổi sang bộ giải tập tích cực như eiquadprog).

### CÒN LỖI CHƯA XONG: node sập sau ~60 giây

```
free(): invalid next size (fast)
```

Hỏng vùng nhớ heap, lặp lại được, luôn quanh ~31000–32000 chu kỳ (~61 s).

Đã thử và **chưa sửa được**:
1. Nghi huỷ đối tượng khi luồng DDS còn chạy → thêm cờ `stopping` + `CloseChannel()`
   trước khi đọc số liệu lần cuối. **Không hết** — lần sau sập giữa chừng chứ
   không phải lúc thoát, nên giả thuyết này sai.
2. Nghi callback chạy chồng nhau (wifi về thành cụm, hàng đợi 10 gói) phá bộ đệm
   dùng chung của `BalanceWBC`/`LegOdometry` → bọc **toàn bộ** callback bằng một
   mutex, và thêm bộ đếm `max_cb` để **đo** thay vì đoán. **Vẫn sập.** Chưa đọc
   được `max_cb` vì log bị lỗi DDS làm ngập rồi sập.

Nên giả thuyết "callback chồng nhau" **chưa được xác nhận cũng chưa bị loại**.

Hướng tiếp theo, theo thứ tự:
1. Chạy dưới `valgrind` hoặc build với `-fsanitize=address` — sẽ chỉ thẳng vào
   dòng ghi tràn, thay vì tiếp tục đoán.
2. Chạy với `--csv` tắt, để loại `ofstream` khỏi danh sách nghi can.
3. Đọc `max_cb` bằng cách ghi ra file thay vì stdout.

Số liệu thời gian ở trên vẫn dùng được: chúng lấy từ 31000 chu kỳ **trước** khi
sập, và bản thân phép đo không liên quan đến chỗ hỏng.

### Gỡ lỗi node sập: bốn giả thuyết, ba đã loại

**1. Huỷ đối tượng khi luồng DDS còn chạy** → thêm cờ `stopping` + `CloseChannel()`.
**Sai.** Lần sau sập giữa chừng chứ không phải lúc thoát.

**2. Callback chạy chồng nhau phá bộ đệm dùng chung của `BalanceWBC`.** Thay vì
đoán, thêm bộ đếm `max_cb` để **đo**. Kết quả: `callback chong nhau toi da 1`.
**Bị bác bỏ bằng số liệu.** (Đồng thời bỏ `--csv` cũng vẫn sập, nên `ofstream`
cũng bị loại. Và số chu kỳ lúc sập không cố định: 31026 / 31982 / 48513.)

**3. AddressSanitizer — và một tạo tác phải tự sửa mình.** Lần chạy đầu ASan báo
ngay `heap-buffer-overflow` trong `Eigen::handmade_aligned_free` ← `ModelTpl::
~ModelTpl` ← `urdf::buildModel` ← `LegOdometry::LegOdometry`. Trông như đã tìm ra.
**Nhưng đó là tạo tác của chính ASan.** Eigen có dòng này trong `Memory.h`:

```c
#if defined(__GLIBC__) && ... && ! defined(__SANITIZE_ADDRESS__) && (EIGEN_DEFAULT_ALIGN_BYTES == 16)
  #define EIGEN_GLIBC_MALLOC_ALREADY_ALIGNED 1
```

Dưới ASan, Eigen **đổi** sang `handmade_aligned_malloc/free`, còn
`libpinocchio_parsers.so` (không ASan) vẫn dùng `malloc` thường. Vùng nhớ do thư
viện cấp phát bị code của ta giải phóng bằng `handmade_aligned_free` — hàm này
đọc 8 byte *phía trước* con trỏ. Đo lại cho chắc:

```
flags: []                                          MALLOC_ALREADY_ALIGNED=1
flags: [-fsanitize=address]                        MALLOC_ALREADY_ALIGNED=0
flags: [-fsanitize=address -DEIGEN_MALLOC_ALREADY_ALIGNED=1]  MALLOC_ALREADY_ALIGNED=1
```

Nên build ASan **bắt buộc** phải ép `-DEIGEN_MALLOC_ALREADY_ALIGNED=1` (đã đưa vào
CMakeLists). Ép xong chạy lại: **200 s / 79394 chu kỳ, ASan không báo lỗi nào**,
tổng kết in đầy đủ. Tức ASan hoặc che mất lỗi (nó thay allocator, vùng đệm đỏ
nuốt phần ghi tràn do code trong `.so` gây ra — ASan không kiểm tra được code
không được instrument), hoặc lỗi phụ thuộc allocator của glibc.

**Bài học:** chạy ASan trên code dùng thư viện dựng sẵn có Eigen thì phải kiểm tra
`EIGEN_MALLOC_ALREADY_ALIGNED` trước, không thì tin nhầm báo cáo đầu tiên.

**4. Tái hiện KHÔNG CẦN ROBOT.** Cho `run_test_vectors --bench` chạy 110000 chu kỳ
với **cả** `LegOdometry` lẫn `BalanceWBC` (phải sửa: bản đầu chỉ gọi odometry ở
lượt ghi cuối nên `--bench 3000` vẫn chỉ gọi nó 55 lần). **Không sập.** Gấp 3.5
lần số chu kỳ đã làm bản sống sập.

Nên phần toán sạch khi chạy một mình. Thứ còn lại chỉ có tầng SDK/DDS. Phép thử
dứt điểm là `--norun` (chỉ nhận và đếm, tắt hẳn odometry/WBC) — **chưa chạy được,
card wifi USB bị rút giữa chừng.**

Nếu `--norun` vẫn sập thì chỗ hỏng nằm trong `libunitree_sdk2.a` / CycloneDDS
dựng sẵn, không phải code của ta — và cách xử lý sẽ khác hẳn (dựng lại CycloneDDS
từ nguồn, hoặc chạy valgrind vốn kiểm được cả code trong `.so`).

### Nguyên nhân gốc: HAI bản CycloneDDS trong một tiến trình

Phép thử `--norun` (chỉ nhận và đếm, tắt hẳn odometry/WBC) **vẫn sập**, rất đều:
~29200 và ~29400 gói ở hai lần chạy. Phần toán được minh oan hoàn toàn.

Không có valgrind (cần sudo), nên dùng gdb lấy backtrace lúc abort:

```
#8  __GI___libc_free ()
#9  ddsi_sertype_unref ()   from /opt/ros/jazzy/lib/x86_64-linux-gnu/libddsc.so.0
#12 org::eclipse::cyclonedds::core::DDScObjectDelegate::~DDScObjectDelegate ()
                             from /home/dung/unitree_sdk2/thirdparty/lib/x86_64/libddscxx.so.0
```

`libddscxx` của SDK (dựng theo CycloneDDS **0.10.2-noshm**) đang gọi vào `libddsc`
**của ROS**. Xác nhận bằng `ldd`:

```
source ROS:        libddsc.so.0   => /opt/ros/jazzy/lib/x86_64-linux-gnu/libddsc.so.0
                   libddscxx.so.0 => /home/dung/unitree_sdk2/thirdparty/.../libddscxx.so.0
uu tien SDK:       ca hai          => /home/dung/unitree_sdk2/thirdparty/...
```

Và **chính mình gây ra**: `run_shadow.sh` có `source /opt/ros/jazzy/setup.bash` để
lấy pinocchio lên `LD_LIBRARY_PATH` — mà thư mục đó cũng chứa `libddsc.so.0`, và
**`LD_LIBRARY_PATH` thắng cả `RPATH`** (binary đã có RPATH trỏ đúng chỗ, vẫn thua).
Hai bản khác layout struct → ghi/đọc tràn → hỏng heap, rồi `free()` bất kỳ sau đó
mới nổ. Vì vậy số chu kỳ lúc sập không cố định và backtrace không chỉ vào đâu cả.

Sửa hai chỗ:
1. `run_shadow.sh` đặt thư mục `thirdparty` của SDK **trước** đường dẫn ROS.
2. `shadow_node.cpp` đọc `/proc/self/maps` lúc khởi động, nếu `libddsc` và
   `libddscxx` đến từ hai thư mục khác nhau thì **in rõ rồi thoát**. Lỗi này tốn
   nhiều giờ vì nó im lặng; giờ không thể dính lại mà không biết.

Kiểm chứng: `--norun` chạy **86798 gói** (trước sập ở ~29200), thoát sạch.

### Số đo cuối trên robot thật

180 giây, 79616 chu kỳ ở ~440 Hz, robot **có bị tác động** (5.4% thời gian một chân):

| | Python | C++ |
|---|---|---|
| TB | — | 0.869 ms |
| p99 | 3.39 ms | **2.053 ms** |
| p999 | — | 3.040 ms |
| xấu nhất | 13.58 ms | **4.272 ms** |

Đuôi co lại ~3 lần. **Nhưng vẫn CHƯA vừa ngân sách**: p99 2.05 ms đã vượt 2.0 ms,
xấu nhất 4.27 ms gấp đôi. Lần chạy 60 s lúc robot đứng yên thì vừa (p99 1.45,
max 2.08) — nhiễu động mới là lúc khó, và đó đúng là lúc cần nhất.

QP chiếm 0.842 trên 0.869 ms, tức **gần như toàn bộ**. Muốn cắt thì phải cắt ở QP:
1. Chuẩn hoá thang bài toán (hàm mục tiêu đang cỡ 1e7) — cũng chính là thứ đã làm
   lệch test vector ở tư thế một chân. Một việc sửa hai chỗ.
2. Hoặc đổi sang bộ giải tập tích cực (eiquadprog — cùng thuật toán với `quadprog`
   của bản Python, `ros-jazzy-eiquadprog` có trong apt nhưng cần sudo để cài).
3. Hoặc hạ xuống 250 Hz.

Cũng lần đầu thấy **QP thất bại 3 lần / 79616** khi robot bị tác động — trước đây
mọi phép thử offline đều 0. Chưa tìm hiểu.

---

## 14. Đổi bộ giải QP: eiquadprog (2026-10-03, tối)

Kế hoạch ban đầu là **chuẩn hoá thang bài toán**. Đo trước khi viết, và **tiền đề
của kế hoạch hoá ra sai**.

### Đo 1: cân bằng Ruiz của ProxSuite không giúp gì

| | vòng lặp QP | lệch `OUT_TAU` | thời gian |
|---|---|---|---|
| mặc định (có Ruiz) | TB 24, max 51 | 1.717e-06 | 0.656 ms |
| `--noprecond` | TB 24, max 47 | **6.200e-07** | 0.685 ms |

Cùng số vòng lặp, mà tắt đi còn **chính xác hơn 2.8 lần**. Nên: (a) ProxSuite
không tự xử lý được chỗ này, (b) số vòng lặp **không** do điều kiện số quyết định.

### Đo 2: thời gian đi đâu

```
pinocchio 0.020 | dung ma tran 0.026 | giai QP 0.599 ms   (92% o QP)
```

Ghép với số vòng lặp (14 vòng → 0.332 ms, 24 vòng → 0.599 ms): **~27 µs mỗi vòng
lặp, gần như không có chi phí cố định.** Thời gian tỉ lệ thẳng với số vòng lặp.

Hai phép đo cộng lại: số vòng lặp ADMM bị **việc dò xem ràng buộc nào đang tích
cực** quyết định (69 bất đẳng thức), không phải điều kiện số. Chuẩn hoá thang
không chạm tới được chỗ đó. Phải đổi **phương pháp**, không phải đổi **đơn vị**.

### eiquadprog

Phương pháp tập tích cực (Goldfarb–Idnani) — **chính thuật toán mà `quadprog` của
bản Python dùng**. Không có trong apt nếu không sudo, nhưng nguồn chỉ một file
`.cpp` phụ thuộc duy nhất Eigen → dựng tại chỗ từ
[github.com/stack-of-tasks/eiquadprog](https://github.com/stack-of-tasks/eiquadprog),
không cần quyền gì. LGPL-3 nên build thành **thư viện chia sẻ riêng** và liên kết
động, không chép nguồn vào repo. (Cần một `config.hpp` rỗng thay cho file vốn do
`jrl-cmakemodules` sinh ra: [third_party_shim/](G1/src/g1_wbc_cpp/third_party_shim/).)

Nó nhận ràng buộc **một phía** (`CI z + ci0 >= 0`) nên phải dựng lại dạng đó —
106 hàng thay vì 69. Chân đang chạm đất thì hàng chặn trên của `fz` được **vô
hiệu bằng 1e9** chứ không bỏ hàng, để số hàng cố định.

### Kết quả — sửa CẢ HAI vấn đề

| bộ vector | bộ giải | giải QP | cả chu kỳ | lệch `OUT_TAU` |
|---|---|---|---|---|
| `real_disturb` | ProxQP | 0.585 ms | 0.633 | 1.717e-06 |
| `real_disturb` | **eiquadprog** | **0.187 ms** | **0.230** | **1.025e-09** |
| `real_quiet` | ProxQP | 0.325 ms | 0.372 | 7.018e-08 |
| `real_quiet` | **eiquadprog** | **0.100 ms** | **0.144** | **9.245e-10** |

Chỗ lệch 1.7e-06 ở tư thế một chân **biến mất** — vì giờ cùng thuật toán với bản
Python nên khớp đến mức máy. Cái mà chuẩn hoá thang định sửa, đổi bộ giải sửa
luôn, và còn nhanh hơn 3.1 lần.

### Trên robot thật, 180 s, 91319 chu kỳ, có tác động (6.8% một chân)

| | Python | ProxQP | **eiquadprog** |
|---|---|---|---|
| TB | — | 0.869 ms | **0.755** |
| p99 | 3.39 ms | 2.053 ms | **1.248** |
| p999 | — | 3.040 ms | **1.448** |
| xấu nhất | 13.58 ms | 4.272 ms | **2.176** |
| QP hỏng | — | 3 | **0** |

**Điều kiện 3 coi như đạt**: p99 và p999 đều trong ngân sách 2.0 ms. Chỉ còn mẫu
xấu nhất 2.176 ms vượt 8.8%, và đó là một mẫu trên 91319.

### Một bài học về PHƯƠNG PHÁP ĐO

Chia nhỏ thời gian khi chạy **sống** (robot đứng yên):

```
pinocchio 0.085 | dung ma tran 0.102 | giai QP 0.370 ms | vong lap TB 5.2
```

so với offline: `0.020 | 0.025 | 0.100`. **Mọi phần đều chậm hơn ~4 lần, đồng
đều** — không phải lỗi bộ giải nào. Bench offline lặp đi lặp lại cùng 55 vector
nên dữ liệu nằm sẵn trong cache; chạy sống thì mỗi chu kỳ chạm dữ liệu mới và
cache bị tiến trình khác đẩy ra trong 1.9 ms giữa hai chu kỳ.

Nghĩa là **`--bench` offline đánh giá thấp đi khoảng 4 lần và không dùng để kết
luận về hạn chót được.** Nó vẫn tốt để *so sánh* hai bộ giải với nhau, nhưng con
số tuyệt đối phải lấy từ robot thật.

### Còn lại

1. Tầng an toàn — vẫn chưa có dòng nào. Giới hạn mô-men riêng, watchdog, tăng dần
   hệ số từ 0, nút dừng khẩn, giàn treo.
2. Ngưỡng test vector vẫn để `1e-4` (trần cho cả hai bộ giải); với eiquadprog
   thực tế đạt `1e-9`, nên có thể siết lại nếu chốt dùng một bộ giải.
3. Chưa đo trên máy tính của robot, mới đo trên laptop.

---

## 15. Tầng an toàn (2026-10-03, tối)

| File | Việc |
|---|---|
| [`include/g1_wbc/safety.hpp`](G1/src/g1_wbc_cpp/include/g1_wbc/safety.hpp) · [`src/safety.cpp`](G1/src/g1_wbc_cpp/src/safety.cpp) | cổng an toàn, không biết gì về DDS |
| [`src/test_safety.cpp`](G1/src/g1_wbc_cpp/src/test_safety.cpp) | 35 phép thử, không cần robot |

**Phạm vi có chủ ý:** phần publish `rt/lowcmd` **chưa viết**. Cổng này nhận mô-men
WBC tính ra và trả về mô-men *được phép* gửi — nhưng không có đường nào từ đó đến
robot. Nhờ vậy chạy được trên robot thật ngay để xem nó chặn những gì, mà vẫn chỉ
đọc. Việc phát lệnh là một quyết định tách bạch.

### Ba nguyên tắc

1. **Mặc định là KHOÁ.** Phải gọi `arm()` tường minh mới mở.
2. **Lỗi thì CHỐT.** Không tự phục hồi. Ra khỏi lỗi chỉ bằng `reset()`, và
   `reset()` đưa về **KHOÁ** chứ không phải đang chạy — nghĩa là sau mỗi lỗi đều
   phải tăng dần lại từ 0.
3. **Cắt bớt không phải là lỗi.** Chạm trần mô-men, chặn tốc độ biến thiên, hệ số
   tăng dần — đều là chuyện bình thường. Lỗi là những điều kiện nói rằng *mô hình
   hoặc phép đo đang sai*, và lúc đó mô-men của ta vô nghĩa.

### Cắt bớt (bình thường)

- trần mô-men riêng: `25%` giới hạn động cơ, trần tuyệt đối `30 Nm` (gối G1 cho
  phép 139 Nm)
- chặn tốc độ biến thiên `150 Nm/s`, **đặt sau** trần — cắt trần trước thì bước
  nhảy còn lại vẫn có thể lớn
- hệ số tăng dần `0 → 1` trong 3 s

### Chốt lỗi

mất `lowstate` > 10 ms · NaN/Inf · quaternion lệch chuẩn > 5% · QP hỏng 3 chu kỳ
**liên tiếp** · trễ chu kỳ 5 lần **liên tiếp** · nghiêng > 20° · CoM lệch > 120 mm
· vận tốc khớp > 12 rad/s · vận tốc thân > 1.5 m/s · mất tiếp xúc một bàn chân

"Liên tiếp" là có chủ ý: một chu kỳ tốt xen vào sẽ đặt lại bộ đếm, và có phép thử
riêng cho điều đó.

### Một điều phải nói rõ về "hành động khi lỗi"

**Mô-men bằng 0 trên một humanoid đang đứng KHÔNG phải là an toàn — nó đổ.** Cho
giảm chấn (`-kd*dq`) đổ chậm hơn nhưng vẫn đổ. Không có lựa chọn nào trong file
này là an toàn thật sự; thứ an toàn thật sự là **giàn treo** và **nút dừng khẩn do
người cầm**. Cổng này chỉ làm cho cú đổ bớt bạo liệt hơn.

### Kiểm thử

35 phép thử, `PASS`. Mỗi luật một phép thử, và có cả phép thử cho điều **ngược
lại** — ví dụ: đã chốt lỗi thì `arm()` phải không làm gì; `reset()` phải về KHOÁ
chứ không phải đang chạy; một chu kỳ tốt phải đặt lại bộ đếm QP hỏng.

Một phép thử hỏng lúc đầu: *"sau reset phải tăng dần lại từ đầu"*. Hỏng ở **bài
thử**, không phải ở cổng — bài đó đặt `ramp_secs = 0` rồi lại đòi thấy trạng thái
`DANG TANG`. Đã sửa bài thử và ghi chú lại lý do ngay tại chỗ.

### Chế độ khô trên robot thật

Node bóng chạy cổng an toàn **mỗi chu kỳ** và báo cáo xem nó *sẽ* chặn những gì.
Khi chốt lỗi thì tự đặt lại để đếm tiếp — **chỉ ở chế độ khô**; điều khiển thật
thì lần ngắt đầu tiên là dừng hẳn.

### Chạy chế độ khô trên robot thật — ba ngưỡng, ba kết luận khác nhau

59 s, 30279 chu kỳ, robot bị tác động (24.9% thời gian một chân).

**1. Mất `lowstate` — ngắt 1.25%. Vấn đề KHÔNG nằm ở ngưỡng.**

Khe thời gian giữa hai chu kỳ có cấu trúc hai đỉnh rõ:

```
p50 1.88 ms | p90 2.15 ms | p99 10.33 ms | p99.9 11.90 ms | max 32.04 ms
nguong 10 ms -> ngat 378 lan (1.25%)
nguong 15 ms -> ngat   1 lan (0.003%)
```

Nới lên 15 ms thì gần như hết ngắt — nhưng đó là làm bộ phát hiện **im lặng**,
không sửa được gì. Một bộ điều khiển 500 Hz mà mù 10–12 ms là mù mất 5–6 chu kỳ.
**Kết luận: điều khiển thật phải qua Ethernet.** Giữ nguyên 10 ms để con số này
tự nói lên điều đó.

Đây là loại kết luận mà chạy khô sinh ra được còn suy luận thì không: trước khi
đo, "wifi chắc đủ dùng" là một giả định hoàn toàn hợp lý — leg odometry và node
bóng đã chạy tốt trên wifi suốt.

**2. Tốc độ biến thiên mô-men — ngưỡng của mình sai hẳn.**

```
|dtau/dt| cua WBC:  p50 417 | p90 2172 | p99 11395 | max 166600 Nm/s
nguong  150 Nm/s -> chan 89.3% chu ky      <- gia tri cu, DOAN ra
nguong 4000 Nm/s -> chan  ~5%              <- da doi sang
```

150 Nm/s không phải "chặn đột biến" mà là **bóp nghẹt bộ điều khiển**. Đã đổi
sang 4000 Nm/s, có ghi cả phân bố đo được ngay tại chỗ khai báo.

Nhưng con số p99 = 11395 Nm/s tự nó đáng lo: trên một chu kỳ 2 ms đó là bước nhảy
22 Nm, ngang với chính độ lớn mô-men. **Lệnh mô-men của ta đang quá giật** để cầm
lái. Rung đo được trong lần chạy này là 15.9 Nm so với 5.6 Nm của Unitree.

**3. Mất tiếp xúc một chân — ngắt 24.4%, và đó KHÔNG phải ngưỡng đặt sai.**

Công thức WBC ràng buộc **cả hai** bàn chân. Một chân rời đất không phải là "khó
hơn một chút" mà là **giả thiết bị vi phạm** — ra ngoài mô hình. Tỉ lệ ngắt 24.4%
khớp đúng với 24.9% thời gian robot ở giai đoạn một chân khi bị đẩy.

Đây là **giới hạn thật của bộ điều khiển hiện nay**, không phải con số cần chỉnh.
Và nó mâu thuẫn với chính mục tiêu "giữ thăng bằng khi bị đẩy": cú đẩy đủ mạnh để
nhấc một chân lên chính là lúc cần bộ điều khiển nhất, mà cũng là lúc nó hết hiệu
lực. Muốn qua được chỗ này phải mở rộng công thức sang tiếp xúc thay đổi, chứ
không phải nới ngưỡng.

**Một cảnh báo về chính số liệu này:** ở chế độ khô, mỗi lần ngắt đều tự đặt lại
rồi tăng dần từ 0, nên thống kê "cắt trần" và "chặn tốc độ" của *cổng* bị méo
(báo 39–52% trong khi tính từ mô-men thô của WBC là 89%). Con số đáng tin là con
số tính từ CSV mô-men thô, không phải con số cổng tự báo.

### Số đo thời gian kèm theo (robot đứng yên, 62525 chu kỳ)

```
ca chu ky: TB 0.553 | p99 0.840 | p999 1.017 | max 2.642 ms
chia nho : pinocchio 0.088 | dung ma tran 0.106 | giai QP 0.322 ms | vong lap TB 1.0
```

### Ethernet — và một kết luận phải rút lại

Cắm cáp vào `enp2s0` (`192.168.123.36`). Chạy khô 60 s, 31053 chu kỳ:

```
ca chu ky: TB 0.551 | p99 0.823 | p999 0.970 | max 1.263 ms
cong an toan (KHO): 0 loai loi
```

**Cổng an toàn không ngắt lần nào.**

So sánh khe thời gian, cùng robot đứng yên, chỉ khác đường truyền:

| | wifi | ethernet |
|---|---|---|
| p50 | 1.76 ms | 1.89 ms |
| p99 | **11.01 ms** | **2.17 ms** |
| max | 26.46 ms | 4.10 ms |

Và tốc độ biến thiên mô-men, vẫn cùng điều kiện đứng yên:

| | wifi | ethernet |
|---|---|---|
| p50 | 473 Nm/s | 300 |
| p90 | **5262** | **566** |
| p99 | **23952** | **844** |
| vượt 4000 Nm/s | 11.22% | **0.00%** |

**Phải rút lại kết luận ở mục trên.** Ở phần phân tích chạy khô trên wifi, đã viết
*"lệnh mô-men của ta đang quá giật để cầm lái"* dựa trên p99 = 11395 Nm/s. Sai.
Với cùng robot làm cùng một việc, qua Ethernet con số đó là **844 Nm/s** — chênh
28 lần. Đó là tạo tác của đường truyền, không phải tính chất của bộ điều khiển.

Cơ chế: khe 10 ms làm trạng thái nhảy một bước lớn, mà bộ lọc vận tốc trong node
lại dùng `dt` cố định `0.002` thay vì khoảng thời gian thật. Một chỗ đáng sửa
riêng, nhưng trên Ethernet thì nó không còn gây hại.

Bài học về phương pháp: hai lần chạy đầu khác nhau **cả hai biến** (đường truyền
*và* có/không bị đẩy), nên không kết luận được gì. Phải lấy bản ghi wifi lúc đứng
yên đã có sẵn rồi phát lại, mới tách được ra.

**Chưa đo được:** nhiễu động thật trên Ethernet. Lần chạy định làm việc đó thì
robot không hề bị tác động — kiểm tra lại số liệu: tốc độ thân max 70 mm/s và CoM
lệch max 4.0 mm, so với 921 mm/s và 78 mm của bản ghi có đẩy thật. Nên ba câu hỏi
sau vẫn còn mở:
1. Tốc độ biến thiên mô-men khi **thực sự** bị đẩy, trên Ethernet, là bao nhiêu?
2. Tỉ lệ ngắt vì mất tiếp xúc một chân khi bị đẩy — có còn 24% không?
3. Rung mô-men so với Unitree khi bị đẩy.

### Chạy khô CÓ ĐẨY trên Ethernet — và trần mô-men

31054 chu kỳ, robot bị đẩy thật (|v| max 2151 mm/s, CoM lệch tới 56.9 mm).

**Ba câu hỏi còn mở, đã trả lời:**

| `\|dtau/dt\|` | p50 | p90 | p99 | max |
|---|---|---|---|---|
| ethernet, đứng yên | 300 | 566 | 844 | 2718 |
| **ethernet, bị đẩy** | 393 | 1142 | **4222** | 67271 |
| wifi, bị đẩy | 417 | 2172 | **11395** | 166601 |

Cả hai đều góp: nhiễu động làm giật thật, wifi nhân thêm ~2.7 lần. Ngưỡng
4000 Nm/s chặn 1.10% — đúng vai trò chặn đột biến.

Ngắt vì mất tiếp xúc 1.34%, khớp đúng 1.4% thời gian một chân. (Lần wifi 24.4%
là vì robot bị đẩy mạnh hơn nhiều, không phải vì đường truyền.) Rung 10.6 Nm so
với Unitree 3.29 Nm.

**Nhưng phát hiện lớn hơn cả ba: trần mô-men chạm 87.9% chu kỳ.**

```
khop         p50     p99     max | Unitree max | tran cu
L_hip_p     33.3    66.7    88.0 |        31.8 |    22.0
L_knee       3.3    46.8    70.9 |        61.8 |    30.0
R_hip_r      2.4    42.3    88.0 |        47.5 |    22.0
```

WBC đòi tới **88.0 Nm** ở hông — **đúng bằng giới hạn động cơ**, tức QP đang bão
hoà chính ràng buộc mô-men của nó. Nới trần tuyệt đối không giải quyết được:

```
tran tuyet doi 30 Nm -> cham 87.9%      tau_scale 0.25 -> cham 87.4%
tran tuyet doi 80 Nm -> cham 87.4%      tau_scale 0.50 -> cham 15.7%
                                        tau_scale 0.70 -> cham  5.8%
```

vì thứ đang chặn là `tau_scale`, không phải trần tuyệt đối.

**Đây không phải lỗi chỉnh số mà là một thông tin thật**, và nó đặt ra mâu thuẫn
phải nói thẳng: *một trần mô-men đủ chặt để thật sự bảo vệ thì cũng đủ chặt để bộ
điều khiển không giữ được thăng bằng.* Biên an toàn phải đến từ **giàn treo, nút
dừng khẩn và việc giới hạn độ mạnh cú đẩy** — không đến từ việc cắt mô-men.

Đã chọn `tau_scale = 0.5`, `tau_abs_max = 60 Nm` cho lần treo giàn đầu tiên.
Kiểm chứng lại trên robot: trần chạm **3%** (trước 82%), `|tau|` gửi max 31.85 Nm
nên không còn bị ghim ở trần, cổng ngắt 0 lần, p99 1.028 ms.

### Bảng năm điều kiện, cập nhật

| | Điều kiện | Tình trạng |
|---|---|---|
| 1 | Công thức đúng | ✅ MuJoCo, khớp lý thuyết capture point |
| 2 | Ước lượng trạng thái đủ tốt | ✅ leg odometry, nhiễu nền 7/5.6/0.8 mm/s |
| 3 | Chạy kịp 2 ms ở 500 Hz | ✅ p99 1.03 ms, xấu nhất 1.59 ms (Ethernet) |
| 4 | Không hỏng giữa chừng | ✅ |
| 5 | Mô-men hợp lý trên dữ liệu thật | 🔶 cùng dấu 63–88%, đòi mô-men cao hơn Unitree đáng kể |
| 6 | Tầng an toàn | ✅ đã có, đã chỉnh bằng số đo thật |

### Hai thứ còn chặn đường gửi lệnh thật

1. **Công thức chỉ đúng khi hai chân chạm đất.** Cú đẩy đủ mạnh để nhấc một chân
   chính là lúc cần bộ điều khiển nhất, mà cũng là lúc nó ra ngoài mô hình.
2. **Chưa có giàn treo và nút dừng khẩn.** Với `tau_scale = 0.5` thì cổng an toàn
   không còn là lớp bảo vệ chính nữa — vật lý mới là.

---

## 16. Vì sao mô-men của ta cao hơn Unitree (2026-10-03, khuya)

### Trước hết, sửa một phép so sánh sai của chính mình

Đã viết *"WBC giữ 33 Nm ở hông trong khi Unitree chỉ ~2 Nm lúc đứng yên"*. Sai:
33 Nm là **p50 của lần chạy có đẩy**, còn 31.8 là **max của Unitree** — so trung
vị bên này với cực đại bên kia, trên hai lần chạy khác nhau.

So lại cùng thống kê, cùng lần chạy (Ethernet, có đẩy):

```
khop      |   ta TB  Unitree TB |  ta max  Uni max
L_hip_p   |    28.3         2.1 |    88.0     31.8
R_hip_p   |    29.1         1.7 |    88.0     28.0
tong 12 khop chan: ta 126.3 | Unitree 53.3 Nm   (ti le p50 2.67)
```

Chênh lệch **vẫn thật và vẫn lớn** — nhưng giờ con số mới đúng.

### Nghi can 1: sai mô hình khối lượng — BỊ LOẠI bằng phép đo

Khi robot đứng yên, tâm áp lực dưới chân phải nằm ngay dưới khối tâm. Tâm áp lực
tính được từ mô-men khớp **chân**, mà phép tính đó chỉ dùng quán tính các khâu
chân — **không dùng mô hình thân trên**. Nên nó là trọng tài độc lập.

```
CoM mo hinh (ngang) : x  +9.9  y  +7.0 mm
Tam ap luc DO duoc  : x +11.1  y +11.1 mm
SAI LECH            : x  -1.2  y  -4.1 mm   (2206 mau)
```

Mô hình khối lượng đã đúng đến vài milimét. Dịch 2 kg đi 10 cm chỉ làm khối tâm
đổi 5.2 mm, nên vị trí tải đặt sai tối đa ~2 cm — trong sai số.

Người dùng cho biết cụm thiết bị ~2 kg chia **hai chỗ**: giữa lưng và sau gáy.
Đã cập nhật [`payload.yaml`](G1/src/g1_leg_odometry/config/payload.yaml) thành hai
mục, và sai lệch x **giảm từ −1.2 xuống +0.1 mm** — mô tả của người dùng chính
xác hơn mô hình cũ.

### Nghi can 2: mốc tư thế cũ — SAI, và sai một cách đáng ghi lại

Đo được: sau các cú đẩy robot ổn định ở tư thế lệch hẳn **−0.085 rad ở hông,
+0.07 rad ở gối, và không bao giờ trở lại**. `q_ref` bị đóng băng từ giây thứ 1,
nên `kp_q × 0.085 = 8.5 rad/s²` liên tục. Câu chuyện rất hợp lý.

Đã viết `update_reference()` cho mốc trôi chậm, quét hằng số thời gian 30/10/3 s.
**Kết quả: không đổi gì cả** — mô-men hông giữ nguyên 26.6 Nm ở mọi giá trị.

Bài học: một giải thích hợp lý, có số liệu ủng hộ (độ lệch tư thế **là** thật),
vẫn có thể sai về **cơ chế**. Giữ lại hàm đó (mặc định tắt) vì độ trôi là có
thật, nhưng nó **không** giải quyết vấn đề mô-men.

### Nguyên nhân thật: QP chọn cách phân bố lực

Tách mô-men thành ba số hạng `tau = M qacc + h − Jc^T f`:

```
khop         M*qacc   h (trong luc)   -Jc^T f | tau tong
L_hip_p         5.5             3.1      28.8 |     27.6
L_hip_r         3.7             0.1      23.6 |     23.4
```

**Toàn bộ** đến từ số hạng lực tiếp xúc. Đứng hai chân là bài toán **vô định tĩnh
học**: nhiều cách phân bố lực cho cùng một hợp lực nhưng mô-men khớp khác hẳn.
Hàm mục tiêu cũ không hề phạt mô-men nên QP chọn bừa một nghiệm trong họ đó.

### Thêm số hạng phạt mô-men `w_tau`

`(w_tau/2)·‖tau‖²`, tức `H += w T'T`, `g += w T'(−h_a)`. Mặc định 0.

Trên bản ghi robot thật có đẩy:

| `w_tau` | hip_p TB | tổng 12 khớp | CoM max | rung | tương quan CoM |
|---|---|---|---|---|---|
| 0 | 26.6 | 160.7 | 72.3 | 17.63 | +0.355 |
| 1 | 17.5 | 134.1 | 72.3 | 15.85 | +0.558 |
| 10 | 12.7 | 113.1 | 72.3 | 13.46 | +0.558 |
| *Unitree* | *2.0* | *53.3* | — | — | *+0.51* |

CoM **không xấu đi chút nào** — đúng như trông đợi khi ta chỉ đang chọn điểm khác
trong không gian null.

Nhưng phát lại không có phản hồi. Ngưỡng đẩy trong MuJoCo:

| `w_tau` | 0 | 1 | 3 | 10 | 30 |
|---|---|---|---|---|---|
| ngưỡng đẩy | **65 N** | 62 N | 58 N | 58 N | 55 N |

**Có đánh đổi thật.** (Lần quét đầu dùng bước 10 N cho bức tranh không đơn điệu và
sai; phải quét bước 3 N mới thấy rõ.)

### Test vector bắt được một thay đổi MÔ HÌNH

Sau khi sửa `payload.yaml`, đối chiếu C++ bỗng lệch 0.97 Nm. Gần như đi sửa nhầm
phần refactor — nhưng bản **Python** cũng lệch đúng y hệt so với file chuẩn, và
bảng đầu ra chỉ thẳng vào `REF_COMDES lech 3.859e-03`: khối tâm đổi 3.9 mm vì
chia lại tải trọng. File chuẩn đã cũ so với mô hình, chứ mã không sai.

Đã sinh lại file chuẩn; C++ khớp lại ở **2.067e-09**.

Quy tắc rút ra: **đổi `payload.yaml` hay URDF thì phải sinh lại test vector**, và
`REF_COMDES` chính là chỗ báo điều đó.

### Chốt `w_tau = 1.0` và kiểm chứng trên phần cứng

Đặt ở cả bốn chỗ ([`wbc.py`](G1/src/g1_wbc/g1_wbc/wbc.py),
[`replay.py`](G1/src/g1_wbc/g1_wbc/replay.py),
[`balance_wbc.hpp`](G1/src/g1_wbc_cpp/include/g1_wbc/balance_wbc.hpp),
[`wbc_shadow_node.py`](G1/src/g1_wbc/g1_wbc/wbc_shadow_node.py)) và ghi lý do kèm
bảng số đo vào [`config/wbc.yaml`](G1/src/g1_wbc/config/wbc.yaml).

Đối chiếu C++ sau khi sinh lại file chuẩn: `OUT_TAU` lệch **1.243e-10**
(`real_disturb`, 41/55 một chân) và **9.970e-11** (`real_quiet`).

**Chạy khô có đẩy trên robot, so với lần `w_tau=0`** (tỉ lệ với Unitree trong
cùng lần chạy, để khử ảnh hưởng của việc hai lần đẩy khác nhau):

| | `w_tau=0` | `w_tau=1` |
|---|---|---|
| một chân / CoM max | 1.4% / 56.9 mm | **5.7%** / 55.3 mm |
| tổng 12 khớp: ta / Unitree | 126.3 / 53.3 = **2.37** | 70.0 / 59.2 = **1.18** |
| hip_pitch: ta / Unitree | 28.7 / 1.9 = **14.86** | 4.3 / 2.6 = **1.63** |
| tương quan hip_pitch ↔ CoM | **−0.084** | **+0.615** |

Và lần `w_tau=1` còn bị đẩy **mạnh hơn**. Tác dụng thật lớn hơn nhiều so với phát
lại dự đoán (26.6 → 17.5 Nm), vì phát lại không có phản hồi: quỹ đạo robot trong
bản ghi là do bộ điều khiển của Unitree tạo ra, không phải do ta.

Con số đáng kể nhất là **tương quan**. Trước: mô-men hông không liên quan gì đến
sai lệch thăng bằng (−0.08) — nó chỉ là lực nội tại vô nghĩa. Sau: +0.615, chặt
hơn cả Unitree (+0.428). Bộ điều khiển bắt đầu hành xử như một bộ giữ thăng bằng
thay vì như một cái kẹp.

Mô-men đỉnh vẫn chạm 88 Nm (giới hạn động cơ) ở hông lúc đẩy mạnh nhất, và cổng
an toàn cắt trần 12% chu kỳ. Nhưng mô-men **duy trì** thì đã hợp lý.

### Hai cái bẫy lộ ra trong lúc chốt

1. **File chuẩn hết hiệu lực khi đổi HỆ SỐ**, không chỉ khi đổi mô hình. Đổi
   `w_tau` thì nghiệm QP đổi. Quy tắc đầy đủ: *đổi mô hình hoặc đổi hệ số thì
   sinh lại test vector.*

2. **Bài kiểm tra MuJoCo âm thầm kiểm chứng sai cấu hình.** `_arg("--wtau", 0.0)`
   nghĩa là chạy không kèm cờ thì test `w_tau=0`, chứ không phải giá trị đã cấu
   hình — suýt báo cáo "PASS ở 65 N" trong khi đó là kết quả của `w_tau=0`. Đã
   sửa để mặc định đọc thẳng từ chữ ký hàm của thư viện, nên bài kiểm tra không
   thể lệch khỏi cấu hình thật nữa.

   Bảng đánh đổi đưa ra để quyết cũng được đo trên **mô hình tải trọng cũ**; đo
   lại trên mô hình hiện tại thì ngưỡng vẫn 65 → 62 N, nhưng đáng lẽ phải đo lại
   trước khi hỏi.

---

## 17. Đường phát lệnh (2026-10-03, tối muộn) — CHƯA GỬI GÌ

| File | Việc |
|---|---|
| [`include/g1_wbc/lowcmd.hpp`](G1/src/g1_wbc_cpp/include/g1_wbc/lowcmd.hpp) · [`src/lowcmd.cpp`](G1/src/g1_wbc_cpp/src/lowcmd.cpp) | publish `rt/lowcmd`, CRC32, `MotionSwitcher::ReleaseMode()` |
| [`src/handover_node.cpp`](G1/src/g1_wbc_cpp/src/handover_node.cpp) | ba giai đoạn bàn giao |

`handover_node` là **chương trình duy nhất trong cả dự án có thể làm robot chuyển
động**. Mọi thứ khác chỉ đọc.

### Ba lớp chặn, phải mở từng cái một

1. `dry` mặc định **true** — dựng bản tin nhưng không publish.
2. Cổng an toàn mặc định **KHOÁ** — mô-men ra bằng 0 dù có publish.
3. `releaseMode()` phải gọi tường minh — trước đó bộ điều khiển của hãng vẫn giữ
   robot và lấn át lệnh của ta.

Với `--send` còn phải gõ đúng `TOI DONG Y`; với `--release-mode` gõ thêm `TAT DI`.

### Một lựa chọn thiết kế đáng nói

Mô-men WBC đi vào `tau_ff`, còn `kp = 0` và `kd` để một giá trị nhỏ **khác 0**.
Nghĩa là **bộ giảm chấn nằm trong động cơ, không nằm trong vòng lặp của ta**. Nếu
tiến trình này chết, treo, hay bị hệ điều hành cho ngủ, động cơ vẫn tự ghì lại —
không phụ thuộc vào việc mã của ta còn chạy hay không.

### Một ẩn số thật sự

**Không tìm thấy watchdog nào trong SDK.** Chưa biết robot làm gì khi lệnh ngừng
đến — giữ nguyên mô-men cuối, hay tự về an toàn? Nếu là vế đầu thì mọi sự cố phần
mềm đều thành sự cố vật lý. Phải thử khi robot đang **treo**.

### Ba giai đoạn

| | Robot | Lệnh | Trả lời câu hỏi gì |
|---|---|---|---|
| `--stage damp` | **treo**, chân không chạm đất | `tau=0, kp=0, kd` nhỏ | robot có nhận bản tin không (CRC, `mode_machine`); nút giảm chấn trên tay cầm còn tác dụng sau `ReleaseMode` không; **robot làm gì khi lệnh ngừng đến**; rút Ethernet thì sao |
| `--stage hold` | **treo** | servo vị trí, `kp` nhỏ | ánh xạ chỉ số khớp có đúng không — sai thứ tự khớp sẽ lộ ngay mà vô hại |
| `--stage wbc` | **đứng bằng chân**, dây chùng | `tau_ff` từ WBC qua cổng an toàn | bài kiểm tra thật |

Giai đoạn `wbc` **phải** để robot đứng bằng chân, vì công thức giả thiết hai chân
chạm đất. Giàn treo lúc đó chỉ để đỡ khi ngã.

**Nói rõ một chỗ:** ở giai đoạn `hold`, mô-men do **động cơ** sinh ra từ `kp`, nên
**cổng an toàn không chặn được**. Bảo vệ duy nhất là `kp` nhỏ và robot đang treo.
Dòng trạng thái in ra nói thẳng điều này thay vì hiển thị trạng thái cổng — xem
mục dưới.

### Chạy thử chế độ khô đã lộ hai lỗi

1. **`ARM` được chấp nhận trước khi chốt tư thế tham chiếu.** Đã chặn.
2. **Dòng trạng thái gây hiểu nhầm**: luôn hiện "cổng an toàn DANG TANG, hệ số 0"
   ở giai đoạn `damp`/`hold`, trong khi cổng **không hề nằm trên đường lệnh** ở
   hai giai đoạn đó. Người vận hành dễ tưởng đang được bảo vệ. Giờ in thẳng
   "cổng an toàn KHÔNG ÁP DỤNG" kèm lý do.

Chế độ khô chạy đủ 8 giây: nhận `lowstate`, chốt tham chiếu, dựng 50 bản tin giảm
chấn lúc thoát, **không bản tin nào ra khỏi máy**.
