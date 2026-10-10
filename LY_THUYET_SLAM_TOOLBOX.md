# Đọc hiểu slam_toolbox: từng hàm một

Code: `g1_ws/src/slam_toolbox` (nhánh `jazzy`). Lõi thuật toán là **Karto SLAM** trong
`lib/karto_sdk/`. Phần còn lại là lớp bọc ROS.

Thứ tự trong bài = thứ tự **một scan đi qua hệ thống**.

```
/scan ─► [0] laserCallback ─► [1] shouldProcessScan ─► [2] addScan
          ═══════════════ ranh giới ROS │ Karto ═══════════════
      ─► [3] Mapper::Process
            ├─ [4] HasMovedEnough
            ├─ [5] ScanMatcher::MatchScan                 ┐
            │     ├─ [6] AddScans / AddScan / FindValidPoints │ FRONT-END
            │     ├─ [7] SmearPoint (CalculateKernel)     │ (scan matching)
            │     ├─ [8] CorrelateScan                    │
            │     │     ├─ [9] ComputeOffsets (GridLookup)│
            │     │     ├─ [10] operator() (song song)    │
            │     │     └─ [11] GetResponse               │
            │     └─ [12] ComputePositional/AngularCovariance ┘
            ├─ [13] AddRunningScan                        ┐
            ├─ [14] AddVertex                             │
            ├─ [15] AddEdges                              │ POSE GRAPH
            │     ├─ [16] LinkScans / AddEdge             │
            │     ├─ [17] LinkChainToScan                 │
            │     ├─ [18] LinkNearChains / FindNearChains │
            │     └─ [19] ComputeWeightedMean             ┘
            └─ [20] TryCloseLoop                          ┐
                  ├─ [21] FindPossibleLoopClosure         │ BACK-END
                  └─ [22] CorrectPoses                    │ (loop closure
                        └─ [23] CeresSolver + PoseGraph2dErrorTerm ┘ + tối ưu)
```

---

## Phần A: Lớp ROS

### [0] `AsynchronousSlamToolbox::laserCallback`
[slam_toolbox_async.cpp:34](g1_ws/src/slam_toolbox/src/slam_toolbox_async.cpp#L34)

Được gọi mỗi khi có một tin nhắn `/scan`.

```cpp
if (!pose_helper_->getOdomPose(pose, scan->header.stamp)) { WARN("Failed to compute odom pose"); return; }
LaserRangeFinder * laser = getLaser(scan);
if (shouldProcessScan(scan, pose)) addScan(laser, scan, pose);
```

1. **Tra TF `odom → base_frame` tại đúng thời điểm của scan** (`header.stamp`), không phải
   thời điểm hiện tại. Nếu TF chưa có (sim chưa chạy, `use_sim_time` sai, `stabilize_frame`
   chết) thì scan **bị bỏ** kèm cảnh báo `Failed to compute odom pose`.
2. `getLaser`: lần đầu tiên tạo một đối tượng `LaserRangeFinder` lưu thông số cảm biến
   (góc min/max, tầm xa, vị trí lắp trên robot).
3. Lọc, rồi đưa vào Karto.

"Async" nghĩa là: nếu đang bận xử lý scan trước thì scan mới **bị bỏ**, không xếp hàng. Bản
`sync` xếp hàng tất cả, chính xác hơn nhưng có thể trễ dần.

### [1] `shouldProcessScan`
[slam_toolbox_common.cpp:759](g1_ws/src/slam_toolbox/src/slam_toolbox_common.cpp#L759)

Bộ lọc **lớp 1**. Scan bị **loại** nếu rơi vào bất kỳ trường hợp nào sau đây:

| Điều kiện loại | Tham số |
|---|---|
| Đang tạm dừng (pause từ RViz) | |
| Không phải scan thứ N | `throttle_scans: 1` (= không bỏ) |
| Chưa đủ thời gian từ scan được nhận trước | `minimum_time_interval: 0.2` s |
| Là 1 trong 5 scan đầu tiên | cứng trong code: "ổn định ban đầu" |
| Đi chưa đủ xa | xem dưới |

**Điểm bất ngờ (dòng 804–813):**
```cpp
if (check_min_dist_and_heading_precisely_) {
    if (dist2 < min_dist2 && heading_diff < min_rotation) return false;
} else if (dist2 < 0.8 * min_dist2) {
    return false;            // CHỈ xét khoảng cách, KHÔNG xét góc!
}
```
Mặc định `check_min_dist_and_heading_precisely = false`, và yaml của bạn không đặt tham số
này. Vì vậy:
- Phải đi ≥ √0.8 × 0.2 ≈ **0.18 m** mới có scan mới.
- **Robot xoay tại chỗ sẽ KHÔNG tạo scan mới**, dù `minimum_travel_heading: 0.2` có trong yaml.

> Sửa lại so với lần giải thích trước: mình nói "đi ≥ 20 cm **hoặc** xoay ≥ 11°". Thực tế
> với cấu hình hiện tại, ở lớp ROS **chỉ khoảng cách** được xét.
>
> Với humanoid điều này quan trọng: G1 hay xoay tại chỗ để đổi hướng. Nếu muốn bản đồ cập
> nhật khi xoay, thêm `check_min_dist_and_heading_precisely: true` vào yaml.

### [2] `SlamToolbox::addScan`
[slam_toolbox_common.cpp:831](g1_ws/src/slam_toolbox/src/slam_toolbox_common.cpp#L831)

- `getLocalizedRangeScan`: đổi `sensor_msgs/LaserScan` sang `karto::LocalizedRangeScan`. Mảng
  khoảng cách được đổi thành các điểm (x, y), gắn kèm **odom pose** của thời điểm đó.
- Khóa `smapper_mutex_`, vì luồng vẽ bản đồ cũng đọc graph.
- Gọi `Mapper::Process` (chế độ mapping bình thường).
  - `ProcessAtDock` / `ProcessAgainstNodesNearBy`: dùng khi bạn bảo "robot đang ở đây" trên
    bản đồ cũ (tiếp tục bản đồ, localization).
- Nếu xử lý thành công: `setTransformFromPoses(corrected, odom)` tính **TF `map → odom`**:
  ```
  map→odom = map→base (corrected pose) × (odom→base)⁻¹ (odom pose)
  ```
  Đây chính là "phần sửa sai" mà SLAM cộng thêm vào odometry.

---

## Phần B: Mapper, nhạc trưởng

### [3] `Mapper::Process`
[Mapper.cpp:2716](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L2716)

Mỗi scan mang **hai pose**:
- **OdometricPose**: pose theo odometry, có trôi, không bao giờ bị sửa.
- **CorrectedPose**: pose SLAM tin là đúng, bị sửa bởi scan matching và loop closure.

Các bước:

**(a) Kiểm tra (2722).** Laser hợp lệ, số tia khớp với cấu hình.

**(b) Khởi tạo lười (2726).** Ở scan đầu tiên mới tạo các scan matcher, vì cần biết tầm xa
của laser.

**(c) Dự đoán (2735–2738).** Bước "predict":
```cpp
Transform lastTransform(pLastScan->GetOdometricPose(), pLastScan->GetCorrectedPose());
pScan->SetCorrectedPose(lastTransform.TransformPose(pScan->GetOdometricPose()));
```
`lastTransform` là phép biến đổi đưa odom pose của scan trước về corrected pose của nó. Áp
phép này lên odom pose của scan mới:

```
corrected_mới ≈ T(odom_cũ → corrected_cũ) · odom_mới
```

Nói cách khác: **"sai số của odom ở scan trước là bao nhiêu thì giả định scan này cũng sai
y như vậy"**. Đây là dự đoán ban đầu cho scan matching. Odom càng tốt thì dự đoán càng gần
và scan matching càng nhẹ.

**(d) Lọc lớp 2 (2741).** `HasMovedEnough`, xem [4].

**(e) Scan matching (2749–2759).** Khớp scan mới với **running scans** (các scan gần đây) để
tìm `bestPose` và `cov`. Sau đó `SetSensorPose(bestPose)` ghi đè corrected pose.

**(f) Lưu (2762).** Cấp ID duy nhất cho scan (`m_NextScanId++`).

**(g) Cập nhật graph (2766–2777).** Thêm đỉnh, thêm cạnh, thêm vào running buffer, thử loop
closure.

**(h) Ghi nhớ (2780).** Scan này thành `LastScan` cho lần sau.

### [4] `HasMovedEnough`
[Mapper.cpp:3147](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L3147)

Bộ lọc **lớp 2**, logic **HOẶC**. Trả về `true` nếu **một trong**:
1. là scan đầu tiên,
2. đã qua ≥ `minimum_time_interval` giây,
3. đã xoay ≥ `minimum_travel_heading`,
4. đã đi ≥ `minimum_travel_distance`.

Vì lớp 1 đã bắt buộc cách nhau ≥ 0.2 s, điều kiện 2 gần như **luôn đúng**. Thực tế lớp 2 hầu
như không lọc thêm gì với cấu hình của bạn: lớp 1 mới là cửa chặn chính.

Lưu ý: so sánh dùng **odometric pose** của cảm biến (`GetSensorAt`), không dùng corrected
pose. Đây là quãng robot "nghĩ" mình đã đi.

---

## Phần C: Front-end, Correlative Scan Matching

**Ý tưởng chung** (Olson, "Real-Time Correlative Scan Matching", 2009):

Thay vì giải tối ưu như ICP, ta **thử mọi pose** (x, y, θ) trong một cửa sổ quanh dự đoán.
Với mỗi pose, xoay và dịch scan mới rồi đặt lên một "bản đồ cục bộ" dựng từ các scan cũ, và
chấm điểm xem các điểm rơi lên chỗ "đen" nhiều không. Pose có điểm cao nhất thắng.

```
  ICP:          đoán ─► gradient ─► gradient ─► ... (có thể kẹt ở cực tiểu cục bộ)
  Correlative:  thử TẤT CẢ ô trong cửa sổ ─► chọn max   (chắc chắn tìm max toàn cục trong cửa sổ)
```

Đổi lại thì tốn tính toán. Karto bù bằng chiến thuật **thô → tinh** và **bảng tra trước**.

### [5] `ScanMatcher::MatchScan`
[Mapper.cpp:535](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L535)

Đầu vào: scan mới (`pScan`) và tập scan tham chiếu (`rBaseScans`). Đầu ra: `rMean` (pose tốt
nhất), `rCovariance`, và điểm `[0,1]`.

**Bước 1 (547–557).** Scan rỗng thì trả về pose dự đoán kèm covariance cực lớn
(`MAX_VARIANCE`), nghĩa là "không biết gì".

**Bước 2 (559–569).** Đặt **lưới tương quan (correlation grid)** sao cho tâm lưới trùng với
pose dự đoán. Lưới này nhỏ, chỉ bao quanh robot: kích thước ≈ 2 × tầm laser.

**Bước 3 (574).** `AddScans(rBaseScans, ...)` vẽ các scan cũ lên lưới, xem [6].

**Bước 4 (577–585).** Tính cửa sổ tìm kiếm:
- kích thước = `correlation_search_space_dimension: 0.5` m, tức ±0.25 m quanh dự đoán,
- bước **thô** = **2 × độ phân giải lưới** (`correlation_search_space_resolution: 0.01` → 2 cm).

**Bước 5 (588–592), khớp thô.** `CorrelateScan` với:
- vị trí: ±0.25 m, bước 2 cm → 26 × 26 vị trí,
- góc: ±`coarse_search_angle_offset` (0.349 rad = ±20°), bước `coarse_angle_resolution`
  (0.0349 rad = 2°) → 21 góc,
- tổng ≈ **14 000 pose** cần chấm điểm.

**Bước 6 (594–619), mở rộng.** Nếu mọi pose đều được 0 điểm (dự đoán sai nặng), nới góc thêm
20°, rồi 40°, 60°. Đây là cơ chế "tự cứu" khi odom trôi về góc. Robot chân lắc nhiều thì hay
cần đến bước này.

**Bước 7 (621–629), khớp tinh.** Tìm lại quanh kết quả thô:
- vị trí: ±1 ô (nửa bước thô), bước 1 cm,
- góc: ±1° (`0.5 * coarse_angle_resolution`), bước `fine_search_angle_offset` (0.00349 rad = 0.2°).

Lưu ý về tên tham số: trong code, `FineSearchAngleOffset` được dùng làm **bước**, còn nửa bước
thô được dùng làm **phạm vi**. Tên tham số trong yaml dễ gây hiểu nhầm.

### [6] `AddScans` → `AddScan` → `FindValidPoints`
[Mapper.cpp:1032](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1032), [1073](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1073), [1113](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1113)

**`AddScans`:** xóa lưới, rồi gọi `AddScan` cho từng scan tham chiếu.

**`AddScan`:** với mỗi điểm hợp lệ:
1. đổi tọa độ thế giới sang ô lưới (`WorldToGrid`),
2. bỏ nếu nằm ngoài lưới,
3. bỏ nếu ô đó đã là `Occupied` (100), tránh làm nhòe lặp,
4. đặt ô = `GridStates_Occupied` (100),
5. `SmearPoint`: làm nhòe xung quanh, xem [7].

**`FindValidPoints`, lọc theo hướng nhìn.** Đây là phần tinh tế nhất.

Vấn đề: một bức tường mỏng nhìn từ hai phía tạo ra hai mặt. Khi robot đứng ở phía A, nó không
thể nhìn thấy mặt phía B. Nếu vẽ cả điểm của mặt B lên lưới, scan matching có thể khớp nhầm.

Cách làm: đi dọc các điểm của scan theo thứ tự tia quét (ngược chiều kim đồng hồ). Với mỗi
cặp điểm liên tiếp cách nhau > 10 cm, tính:
```
ss = định thức (viewPoint→firstPoint, viewPoint→currentPoint)
```
- `ss ≥ 0`: điểm đi **ngược chiều kim đồng hồ** khi nhìn từ vị trí robot hiện tại. Đoạn bề mặt
  này **quay mặt về phía robot**, nên giữ.
- `ss < 0`: đoạn bề mặt **quay lưng** về phía robot, nên bỏ.

`trailingPointIter` là con trỏ "đi sau", dùng để thêm cả cụm điểm giữa hai mốc kiểm tra.

### [7] `SmearPoint` và `CalculateKernel`
[Mapper.h:1152](g1_ws/src/slam_toolbox/lib/karto_sdk/include/karto_sdk/Mapper.h#L1152), [Mapper.h:1213](g1_ws/src/slam_toolbox/lib/karto_sdk/include/karto_sdk/Mapper.h#L1213)

**`CalculateKernel`** (chạy một lần khi tạo lưới): tạo một **nhân Gaussian 2D**:
```
kernel[i][j] = round( 100 · exp(-½ · (khoảng_cách / σ)²) )
```
với σ = `correlation_search_space_smear_deviation: 0.1` m. Kích thước nhân ≈ ±2σ. σ phải nằm
trong [0.5, 10] × độ phân giải, nếu không sẽ báo lỗi.

**`SmearPoint`:** đặt nhân lên quanh ô bị chiếm, mỗi ô lấy **max** (không cộng dồn) giữa giá
trị cũ và giá trị nhân.

```
Trước smear:          Sau smear (σ lớn):
. . . . .             . 14 32 14 .
. . . . .             14 61 88 61 14
. . 100 . .    ──►    32 88 100 88 32
. . . . .             14 61 88 61 14
. . . . .             . 14 32 14 .
```

**Tại sao cần làm nhòe?** Nếu không làm nhòe, lệch 1 ô là điểm rơi từ 100 về 0: hàm điểm
**nhọn như kim**, khớp thô bước 2 cm rất dễ bỏ qua đỉnh. Làm nhòe biến mỗi điểm thành một
"ngọn đồi", nên:
- khớp thô vẫn thấy được dốc dẫn tới đỉnh,
- chịu được nhiễu đo và sai số tô lưới.

Thực chất đây là **mô hình cảm biến xác suất**: điểm thật nằm đâu đó quanh vị trí đo, theo
phân phối Gaussian.

### [8] `CorrelateScan`
[Mapper.cpp:712](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L712)

Hàm thực hiện **một lượt** tìm kiếm (gọi một lần cho thô, một lần cho tinh).

**(722–723) Bảng tra góc:** `ComputeOffsets` tính trước, cho mỗi góc thử, vị trí tương đối của
mọi điểm laser trên lưới, xem [9].

**(726–732)** Chỉ ở lượt thô: xóa lưới `m_pSearchSpaceProbs`. Lưới này lưu "điểm cao nhất tại
mỗi vị trí (x, y), lấy trên mọi góc" và về sau dùng để tính covariance.

**(736–756) Danh sách ứng viên:** `m_xPoses`, `m_yPoses` (độ lệch so với tâm), `nAngles`.

**(761)** Cấp phát mảng kết quả `m_pPoseResponse` gồm nX × nY × nAngles phần tử, mỗi phần tử là
(điểm, pose).

**(773) Chạy song song:**
```cpp
tbb::parallel_for_each(m_yPoses, (*this));
```
Intel TBB chia các **hàng y** cho các nhân CPU. Mỗi luồng gọi `operator()(y)`, xem [10].

**(776–800) Tìm điểm cao nhất**, đồng thời ghi vào `m_pSearchSpaceProbs` điểm lớn nhất tại mỗi
ô (x, y).

**(803–829) Lấy trung bình các pose có điểm bằng max:**
```cpp
thetaX += cos(heading);  thetaY += sin(heading);
averagePose = Pose2(averagePosition, atan2(thetaY, thetaX));
```
Khi nhiều pose hòa điểm (hành lang dài, đối xứng), lấy tâm của chúng. **Góc được trung bình
qua vector đơn vị**, không cộng trực tiếp, vì trung bình của +179° và −179° phải là 180° chứ
không phải 0°.

**(840–846) Covariance:**
- lượt thô → `ComputePositionalCovariance` (x, y),
- lượt tinh → `ComputeAngularCovariance` (θ).

### [9] `GridLookup::ComputeOffsets`
[Karto.h:6846](g1_ws/src/slam_toolbox/lib/karto_sdk/include/karto_sdk/Karto.h#L6846)

**Mẹo tối ưu tốc độ quan trọng nhất.**

Quan sát: khi dịch scan đi (x, y), vị trí tương đối giữa các điểm **không đổi**. Chỉ khi
**xoay** thì chúng mới đổi. Vì vậy, với **mỗi góc** θ:
1. xoay mọi điểm của scan (đang ở tọa độ cục bộ của robot) đi θ:
   ```
   x' = cos θ · x − sin θ · y
   y' = sin θ · x + cos θ · y
   ```
2. đổi sang **chỉ số mảng 1 chiều** của lưới (`GridIndex`), lưu vào bảng.
3. Tia NaN/Inf được đánh dấu `INVALID_SCAN`.

Sau đó, thử một vị trí (x, y) chỉ là: `chỉ_số_điểm = chỉ_số_vị_trí + offset_tra_bảng`. Không
cần sin/cos, không cần nhân ma trận, chỉ cần **cộng số nguyên**.

Chi phí: sin/cos tính **nAngles × nPoints** lần, thay vì **nX × nY × nAngles × nPoints** lần.
Nhanh hơn khoảng 676 lần với cửa sổ thô.

### [10] `ScanMatcher::operator()`
[Mapper.cpp:641](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L641)

Một luồng xử lý **một hàng y**:
```
for x in m_xPoses:
    gridIndex = ô của vị trí (tâm + x, tâm + y)
    for mỗi góc:
        response = GetResponse(angleIndex, gridIndex)
        nếu doPenalize: response *= distancePenalty * anglePenalty
        lưu (response, pose) vào m_pPoseResponse[(y·nX + x)·nAngles + góc]
```

**Phạt (671–685):** kết hợp với odometry, giống một **prior Gaussian**:
```
distancePenalty = 1 − 0.2 · (dx² + dy²) / distance_variance_penalty      (≥ 0.5)
anglePenalty    = 1 − 0.2 · (dθ²)       / angle_variance_penalty         (≥ 0.9)
```
Pose càng xa dự đoán của odom thì càng bị trừ điểm. Nghĩa là: "nếu hai chỗ khớp tốt như nhau,
tin chỗ gần với odom hơn". Đây là chỗ **odom và laser được trộn** ở front-end.

Trong yaml của bạn: `distance_variance_penalty: 0.5`, `angle_variance_penalty: 1.0`. Tăng các
giá trị này = tin odom **ít** hơn.

Mỗi phần tử ghi vào **chỉ số riêng** của mảng, nên các luồng không đụng nhau và không cần
khóa.

### [11] `GetResponse`, vòng lặp nóng nhất
[Mapper.cpp:1172](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1172)

```cpp
pByte = lưới + gridPositionIndex;                 // con trỏ đến ô của vị trí đang thử
for i in điểm:
    if ngoài lưới hoặc INVALID_SCAN: continue
    response += pByte[offset[i]];                 // đọc giá trị lưới (0..100) tại điểm i
response /= (nPoints * 100);                      // chuẩn hóa về [0, 1]
```

**Ý nghĩa:** điểm = **trung bình độ "đen"** của lưới tại vị trí các điểm laser sau khi đặt
scan vào pose đang thử.
- 1.0: mọi điểm rơi đúng tâm của điểm cũ.
- 0.0: không điểm nào chạm gì.

Về mặt xác suất, đây xấp xỉ **log-likelihood** p(scan | pose, bản đồ cục bộ).

**Câu hỏi tự kiểm tra (bug pelvis nghiêng):** mặt sàn tạo thành vành tròn bán kính ≈
`range_max`. Xoay scan bao nhiêu độ thì các điểm của vành tròn vẫn nằm trên vành tròn cũ, nên
`response` **không đổi theo góc**. Vành tròn chiếm phần lớn số điểm, nên chênh lệch điểm giữa
các góc rất nhỏ. Covariance góc ([12]) phình to, và scan matching mất khả năng xác định hướng.

### [12] `ComputePositionalCovariance` và `ComputeAngularCovariance`
[Mapper.cpp:874](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L874), [977](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L977)

**Ý tưởng:** độ không chắc chắn = **độ rộng của vùng điểm cao**.

```
Đỉnh nhọn (góc phòng):      Đỉnh dẹt dài (hành lang):
     ▲                            ▁▂▃▄▅▅▅▅▅▄▃▂▁
    ▲█▲      cov nhỏ                    cov lớn theo chiều dọc hành lang
```

**Vị trí (thô):** duyệt mọi ô (x, y) trong cửa sổ. Lấy những ô có điểm ≥ `best − 0.1`, rồi tính
**phương sai có trọng số**:
```
σ²_xx = Σ r·(x − x*)² / Σ r
σ²_xy = Σ r·(x − x*)(y − y*) / Σ r
σ²_yy = Σ r·(y − y*)² / Σ r
```
- Chặn dưới: σ² ≥ 0.1 · (bước)², tránh quá tự tin.
- Nhân với `1 / bestResponse`: khớp tệ (điểm thấp) thì covariance to ra.
- σ²_xy ≠ 0 nghĩa là elip lệch, ví dụ hành lang chạy chéo.

**Góc (tinh):** giữ vị trí tốt nhất, quét mọi góc, và tính tương tự:
```
σ²_θθ = Σ r·(θ − θ*)² / Σ r
```

Covariance này rất quan trọng: nó trở thành **trọng số của cạnh** trong pose graph ([16],
[23]). Cạnh có covariance nhỏ thì Ceres "tin" nhiều hơn khi tối ưu.

---

## Phần D: Xây pose graph

**Pose graph:**
- **Đỉnh** = pose của một scan (x, y, θ).
- **Cạnh** = ràng buộc tương đối "scan B nằm ở vị trí Δ so với scan A, với độ tin cậy Σ⁻¹".

### [13] `AddRunningScan`
[Mapper.cpp:182](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L182)

**Running scans** = "bản đồ cục bộ" dùng cho scan matching tuần tự.
```cpp
m_RunningScans.push_back(pScan);
while (size > scan_buffer_size  ||  khoảng_cách(đầu, cuối) > scan_buffer_maximum_scan_distance)
    xóa scan cũ nhất
```
Với yaml của bạn: tối đa 10 scan, và khoảng cách đầu–cuối ≤ 10 m. Đây là một **cửa sổ trượt**.

### [14] `MapperGraph::AddVertex`
[Mapper.cpp:1418](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1418)

Tạo một đỉnh chứa scan, thêm vào graph, và gọi `CeresSolver::AddNode`. Ceres lưu một
`Eigen::Vector3d(x, y, θ)`: đây là **biến số cần tối ưu**, khởi tạo bằng corrected pose hiện
tại.

### [15] `MapperGraph::AddEdges`
[Mapper.cpp:1434](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1434)

Thêm **3 loại cạnh** cho scan mới:

**(1) Cạnh tuần tự (1441–1449):** nối với scan **ngay trước** (ID − 1), dùng kết quả scan
matching ở [5]. Đây là "xương sống" odometry của graph.

**(2a) Scan đầu tiên của một robot (1455–1483):** chỉ có ở chế độ nhiều robot. Khớp với scan
của robot khác để nối hai bản đồ.

**(2b) Nối với running scans (1484–1490):** `LinkChainToScan(running scans)` nối với scan **gần
nhất** trong cửa sổ trượt. Thường nó trùng với scan ngay trước, nên `AddEdge` thấy cạnh đã tồn
tại và bỏ qua.

**(3) Nối với các chuỗi gần (1493):** `LinkNearChains`, xem [18]. Đây là "loop closure nhỏ": khi
robot đi qua lại một chỗ vừa đi cách đây không lâu.

**(4) Gộp (1495–1497):** mỗi lần khớp cho ra một ước lượng pose kèm covariance. Gộp tất cả bằng
`ComputeWeightedMean`, xem [19].

### [16] `LinkScans` và `AddEdge`
[Mapper.cpp:1619](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1619), [1584](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1584)

**`AddEdge`:** tìm 2 đỉnh. Nếu đã có cạnh A→B thì trả về cạnh cũ (`isNewEdge = false`), nếu
chưa thì tạo mới.

**`LinkScans`:** chỉ khi cạnh **mới**:
```cpp
pEdge->SetLabel(new LinkInfo(A.corrected, B tại rMean, rCovariance));
m_pScanOptimizer->AddConstraint(pEdge);
```
`LinkInfo` tính và lưu **độ chênh tương đối** Δ = pose_B biểu diễn trong hệ tọa độ của A. Đây
là "phép đo" của cạnh. Sau đó cạnh được đưa sang Ceres thành một **residual block**, xem [23].

Vì sao lưu **tương đối** chứ không tuyệt đối? Phép đo của cảm biến luôn là tương đối ("B ở
trước A 0.3 m"). Khi graph được tối ưu, mọi pose tuyệt đối đều dịch chuyển, nhưng phép đo
tương đối thì vẫn giữ nguyên.

### [17] `LinkChainToScan`
[Mapper.cpp:1663](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1663)

Được khớp với **cả một chuỗi** scan, nhưng chỉ tạo **một cạnh** tới scan gần nhất trong chuỗi
(`GetClosestScanToPose`). Cạnh này chỉ được tạo nếu scan gần nhất cách ≤
`link_scan_maximum_distance` (1.5 m).

`use_scan_barycenter: true`: dùng **trọng tâm của các điểm laser** thay vì vị trí robot để đo
khoảng cách. Hai scan "nhìn" cùng một vùng thì trọng tâm gần nhau, kể cả khi vị trí robot hơi
khác.

### [18] `LinkNearChains` và `FindNearChains`
[Mapper.cpp:1639](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1639), [1683](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1683)

**`FindNearChains`:**
1. `FindNearLinkedScans`: **duyệt graph** (BFS qua các cạnh) từ scan mới, lấy các scan cách
   ≤ 1.5 m. Chú ý: đây là các scan **có đường đi trong graph** tới scan mới, không phải mọi
   scan nằm gần về mặt không gian.
2. Với mỗi scan gần đó, mở rộng về **trước và sau theo ID** chừng nào còn nằm trong 1.5 m. Kết
   quả là một **chuỗi** scan liên tiếp (một đoạn đường cũ).
3. Chuỗi nào **chứa chính scan mới** thì không hợp lệ (đó là đoạn đường hiện tại), loại bỏ.

**`LinkNearChains`:** với mỗi chuỗi đủ dài (≥ `loop_match_minimum_chain_size` = 10), khớp scan
mới với chuỗi. Nếu điểm > `link_match_minimum_response_fine` (0.1) thì thêm cạnh.

**Khác loop closure [20] thế nào?** Ở đây chỉ xét các scan **đã nối trong graph** và **gần**
(1.5 m). Loop closure xét **mọi** scan, trong bán kính lớn hơn (3 m), và kiểm tra khắt khe hơn
nhiều.

### [19] `ComputeWeightedMean`
[Mapper.cpp:1914](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1914)

Gộp nhiều ước lượng pose (μᵢ, Σᵢ) thành một, giống bước **cập nhật của bộ lọc Kalman**:
```
W_i = (Σ Σⱼ⁻¹)⁻¹ · Σᵢ⁻¹          ← trọng số = độ tin cậy tương đối
μ   = Σ W_i · μᵢ
θ   = atan2(mean sin θᵢ, mean cos θᵢ)   ← góc lấy trung bình vector, KHÔNG có trọng số
```
Ước lượng nào có covariance nhỏ (tự tin) thì kéo kết quả về phía nó mạnh hơn.

Điểm yếu nhỏ đáng chú ý: góc được lấy trung bình **đều**, bỏ qua covariance góc. Đây là một
chỗ bạn có thể thử cải tiến.

---

## Phần E: Back-end, loop closure và tối ưu

### [20] `TryCloseLoop`
[Mapper.cpp:1500](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1500)

```
while (còn chuỗi ứng viên):
    ① khớp THÔ với m_pLoopScanMatcher (cửa sổ LỚN), không phạt, không tinh
    ② cổng 1: coarse > 0.35  VÀ  σ²_x < 3.0  VÀ  σ²_y < 3.0 ?
    ③ khớp TINH với m_pSequentialScanMatcher, xuất phát từ kết quả thô
    ④ cổng 2: fine ≥ 0.45 ?   không → "REJECTED!"
    ⑤ CHẤP NHẬN: sửa pose, thêm cạnh loop, CorrectPoses() → Ceres
    tìm chuỗi ứng viên tiếp theo
```

Chi tiết:
- **① Matcher riêng cho loop:** `loop_search_space_dimension: 8.0` m, tức tìm ±4 m. Khi đi một
  vòng lớn, odom có thể đã trôi vài mét, nên cửa sổ phải lớn. Đổi lại độ phân giải là 5 cm,
  thô hơn. `doPenalize = false` vì lúc này **không tin** odom nữa: mục đích chính là sửa phần
  trôi của nó.
- **② Hai điều kiện:** điểm phải cao, **và** phải chắc chắn. Hành lang dài có thể cho điểm cao
  nhưng σ² dọc hành lang rất lớn, và trường hợp này phải bị loại.
- **③** `tmpScan` là bản sao để thử, chưa ghi đè scan thật.
- **⑤** Sau `CorrectPoses`, các pose thay đổi, nên `FindPossibleLoopClosure` tiếp tục tìm từ
  `scanIndex` để thử các vòng khác.

Các thông báo `FireLoopClosureCheck(...)` là chỗ bạn có thể in log, sẽ hiện ra
`COARSE RESPONSE`, `FINE RESPONSE` và `REJECTED!`.

**Tại sao khắt khe vậy?** Một cạnh loop **sai** sẽ kéo méo **toàn bộ** bản đồ, và Ceres không
có cách gỡ ra (vì `ceres_loss_function: None`, xem [23]). Thà bỏ sót còn hơn nhận nhầm.

### [21] `FindPossibleLoopClosure`
[Mapper.cpp:1960](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L1960)

1. `nearLinkedScans` = các scan có **đường đi trong graph** tới scan mới, trong bán kính 3 m.
   Đây là các scan "đã biết là gần", không cần loop closure.
2. Duyệt **mọi** scan theo ID, bắt đầu từ `rStartNum`:
   - **Gần** (≤ `loop_search_maximum_distance` = 3 m):
     - nếu thuộc `nearLinkedScans` → **xóa chuỗi** (đoạn này đã được nối),
     - không thì thêm vào chuỗi.
   - **Xa:** nếu chuỗi đã ≥ 10 scan thì **trả về chuỗi**. Không thì xóa và tiếp tục.

Kết quả là một đoạn đường cũ **liên tục**, **gần về không gian** nhưng **xa trong graph**.
Đó đúng là định nghĩa của "quay lại chỗ cũ".

**Tại sao cần ≥ 10 scan?** Khớp với một scan đơn lẻ dễ nhầm. Khớp với 10 scan liên tiếp nghĩa
là khớp với một "bản đồ cục bộ" phong phú hơn, nên đáng tin hơn.

### [22] `CorrectPoses`
[Mapper.cpp:2012](g1_ws/src/slam_toolbox/lib/karto_sdk/src/Mapper.cpp#L2012)

```cpp
pSolver->Compute();                                     // Ceres tối ưu
for (id, pose) in pSolver->GetCorrections():
    scan->SetCorrectedPoseAndUpdate(pose);              // ghi đè corrected pose của MỌI scan
pSolver->Clear();
```
Đây là lúc bản đồ "giật" một cái rồi thẳng lại trong RViz.

### [23] `CeresSolver` và `PoseGraph2dErrorTerm`
[ceres_solver.cpp:214](g1_ws/src/slam_toolbox/solvers/ceres_solver.cpp#L214), [ceres_utils.h:80](g1_ws/src/slam_toolbox/solvers/ceres_utils.h#L80)

**`AddConstraint` ([339](g1_ws/src/slam_toolbox/solvers/ceres_solver.cpp#L339)):**
1. Lấy Δ = (Δx, Δy, Δθ) từ `LinkInfo`.
2. Ma trận thông tin Ω = Σ⁻¹, rồi **phân rã Cholesky** để lấy √Ω.
3. Tạo một residual block nối 6 biến (x_a, y_a, θ_a, x_b, y_b, θ_b).
4. `AngleManifold`: báo cho Ceres rằng θ là góc, nên khi cập nhật phải chuẩn hóa về [−π, π].

**Hàm sai số (`PoseGraph2dErrorTerm::operator()`):**
```
r_xy = R(θ_a)ᵀ · (p_b − p_a) − Δp       ← vị trí B nhìn từ A, trừ đi phép đo
r_θ  = normalize((θ_b − θ_a) − Δθ)
r    = √Ω · r                            ← "làm trắng" theo độ tin cậy
```
Nghĩa là: "với các pose hiện tại, B nằm ở đâu so với A? Lệch bao nhiêu so với điều scan
matching đã đo?"

**Bài toán tối ưu:**
```
min  Σ_cạnh  ‖ √Ω_ij · r_ij(x_i, x_j) ‖²     =  Σ rᵀ Ω r   (khoảng cách Mahalanobis)
```
Đây đúng là bài toán **pose graph SLAM** trong lý thuyết. Ceres giải bằng **Levenberg–Marquardt**
(`ceres_trust_strategy`), mỗi bước giải hệ tuyến tính thưa bằng `SPARSE_NORMAL_CHOLESKY`.
`AutoDiffCostFunction` lo tính Jacobian tự động.

**`Compute()`:**
- **Cố định đỉnh đầu tiên** (`SetParameterBlockConstant`). Nếu không làm vậy, cả bản đồ có thể
  trôi tự do (bài toán có 3 bậc tự do thừa: dịch x, y và xoay), vì mọi cạnh chỉ là đo tương
  đối.
- `ceres::Solve`, rồi chép kết quả vào `corrections_`.

**`ceres_loss_function: None`** (trong yaml): dùng bình phương thuần. Một cạnh loop **sai** sẽ
có residual khổng lồ và kéo cả bản đồ theo. Nếu đặt `HuberLoss` hoặc `CauchyLoss`, cạnh sai sẽ
bị giảm ảnh hưởng (robust kernel), đổi lại tối ưu hội tụ chậm hơn một chút.

---

## Tóm tắt: tham số yaml ↔ hàm

| Tham số | Hàm dùng | Tác dụng |
|---|---|---|
| `minimum_time_interval`, `minimum_travel_distance` | [1] shouldProcessScan, [4] | Lọc scan |
| `check_min_dist_and_heading_precisely` (chưa đặt) | [1] | Bật xét góc khi xoay tại chỗ |
| `correlation_search_space_dimension/resolution` | [5] | Cửa sổ và bước của matcher tuần tự |
| `correlation_search_space_smear_deviation` | [7] | σ làm nhòe |
| `coarse_search_angle_offset`, `coarse_angle_resolution` | [5], [8] | Phạm vi và bước góc thô |
| `fine_search_angle_offset` | [5] | **Bước** góc tinh |
| `distance/angle_variance_penalty` | [10] | Mức tin odom |
| `scan_buffer_size`, `scan_buffer_maximum_scan_distance` | [13] | Cửa sổ trượt running scans |
| `link_scan_maximum_distance`, `link_match_minimum_response_fine` | [17], [18] | Cạnh tới chuỗi gần |
| `loop_search_maximum_distance`, `loop_match_minimum_chain_size` | [21] | Tìm ứng viên loop |
| `loop_search_space_*` | [20] | Cửa sổ của matcher loop |
| `loop_match_minimum_response_coarse/fine`, `loop_match_maximum_variance_coarse` | [20] | Hai cổng chấp nhận loop |
| `ceres_*` | [23] | Bộ giải và hàm loss |

## Bài tập đề xuất

1. **In log loop closure:** trong [20], thêm `std::cout` (hoặc bật `debug_logging: true`) để
   thấy coarse/fine response. Chạy chặng A, đi một vòng kín và ghi lại các con số.
2. **Xoay tại chỗ:** cho robot xoay 360° tại chỗ, đếm số scan được thêm vào. Sau đó bật
   `check_min_dist_and_heading_precisely: true` và so sánh.
3. **Robust loss:** đặt `ceres_loss_function: HuberLoss`, cố tình hạ ngưỡng loop xuống thấp
   (ví dụ 0.2) để tạo loop sai. So sánh bản đồ có và không có Huber.
4. **Cải tiến [19]:** cho góc được lấy trung bình có trọng số theo σ²_θθ.
