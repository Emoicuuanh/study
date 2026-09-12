# Câu hỏi phỏng vấn Humanoid Robotics

> File tổng hợp sống — cập nhật dần theo tiến độ khóa học.
> Mỗi câu có: câu hỏi, gợi ý trả lời ngắn, và bài học/file code liên quan.
> Cách ôn: che phần trả lời, tự nói thành lời như đang phỏng vấn thật.

---

## 1. Nền tảng biểu diễn trạng thái (tuần 1–2)

### 1.1. Floating base là gì? Vì sao humanoid cần nó còn robot tay máy/AGV thì không?
**Trả lời:** Humanoid không bị gắn cố định xuống đất — thân robot có thể trôi,
nghiêng, ngã tự do trong không gian 3D. Vì vậy trạng thái cần thêm 6 DOF
"ảo" (3 vị trí + 3 hướng) cho thân, gọi là floating base. Tay máy công nghiệp
bắt vít xuống sàn nên không cần; AGV chỉ cần (x, y, yaw) vì dính trên mặt phẳng.
Hệ quả quan trọng: 6 DOF này **không có động cơ** (underactuated) — muốn di
chuyển thân phải "mượn" lực tiếp xúc chân–sàn.
*Liên quan: `week1_inspect_g1.py` — joint đầu tiên loại `free`.*

### 1.2. Vì sao qpos có 36 phần tử mà qvel chỉ có 35?
**Trả lời:** Hướng xoay của thân được lưu bằng quaternion (4 số, ràng buộc
chuẩn hóa = 1) nhưng vận tốc góc chỉ cần 3 số. Quaternion dùng 4 số để tránh
singularity của Euler angles (gimbal lock) và nội suy mượt.

### 1.3. Kể một bug thực tế liên quan đến quaternion convention.
**Trả lời:** MuJoCo lưu quaternion theo thứ tự (w,x,y,z), Pinocchio và ROS
theo (x,y,z,w). Chuyển dữ liệu giữa hai hệ mà quên đảo thứ tự → FK sai hoàn
toàn nhưng code không báo lỗi. Cách phòng: viết unit test so sánh FK của hai
thư viện trên cùng một tư thế (chênh lệch phải = 0).
*Liên quan: bài FK cross-check tuần 4 — hai thư viện trùng đến 0.000mm.*

---

## 2. Actuator & điều khiển khớp (tuần 2)

### 2.1. Ba chế độ lệnh của động cơ robot là gì? Hiểu sai chế độ gây hậu quả gì?
**Trả lời:** Torque (ra lực thô), velocity (giữ tốc độ), position (servo tự
đến đích — thường có PD chạy ngầm bên trong với gain Kp/Kd cấu hình sẵn).
Gửi giá trị torque vào interface position → con số bị hiểu nhầm thành góc
mục tiêu (ví dụ torque 15 Nm → "bẻ khớp tới 15 rad ≈ 860°") → robot vặn
vẹo/sập. Trên robot thật (Unitree SDK) cũng y hệt — luôn đọc kỹ actuator
interface trước khi gửi lệnh.
*Liên quan: bug đầu tiên của khóa học, `week2_pd_stand.py`.*

### 2.2. Giải thích PD control. Vai trò vật lý của Kp và Kd?
**Trả lời:** τ = Kp(q_ref − q) − Kd·q̇. Kp như lò xo kéo khớp về mục tiêu
(cứng hơn = bám sát hơn nhưng dễ rung), Kd như giảm chấn hãm dao động
(lớn quá = chậm chạp, ì). Không có Kd, hệ Kp thuần dao động không tắt.

### 2.3. Robot đứng bằng PD vị trí khớp thuần túy — vì sao bị đẩy mạnh là ngã?
**Trả lời:** PD từng khớp chỉ biết "giữ góc của mình", không khớp nào biết
cả robot đang nghiêng — robot cứng như bức tượng. Bằng chứng thí nghiệm:
robot nằm sõng soài trên sàn mà lệch khớp chỉ 0.011 rad (ngã trong tư thế
đứng nghiêm hoàn hảo). Muốn chống đẩy phải có feedback trạng thái thân
(IMU) → điều chỉnh mục tiêu: nhún gối, vung tay, bước chân ra (capture point).
*Liên quan: thí nghiệm đẩy — G1 chịu được 30N/0.2s, ngã ở 40N.*

### 2.4. Vì sao control loop của humanoid phải chạy 500Hz–1kHz và cần real-time?
**Trả lời:** Động lực học ngã rất nhanh (thời gian đổ ~vài trăm ms), trễ một
chu kỳ có thể mất thăng bằng. Real-time nghĩa là deadline được đảm bảo —
miss deadline trên robot 33kg đang bước là tai nạn. Đây là lý do JD yêu cầu
C++ real-time (pre-allocate memory, lock-free, không malloc trong loop).

---

## 3. Sinh quỹ đạo chuyển động (tuần 3)

### 3.1. Chuyển động robot được tạo ra như thế nào từ position control?
**Trả lời:** Mọi chuyển động = mục tiêu khớp thay đổi theo thời gian q_ref(t),
cập nhật mỗi chu kỳ điều khiển, để PD đuổi theo. Bộ sinh q_ref(t) gọi là
trajectory/pattern generator. Đi bộ, squat, vẫy tay chỉ khác nhau ở hàm này.
*Liên quan: `week3_squat.py` — một dòng khác biệt so với đứng yên.*

### 3.2. Vì sao dùng A(1−cos(ωt))/2 thay vì A·sin(ωt) khi khởi động chuyển động?
**Trả lời:** (1−cos)/2 có giá trị VÀ đạo hàm đều = 0 tại t=0 → khởi động êm.
sin có vận tốc ban đầu ≠ 0 → cú giật ngay giây đầu (đã thấy bằng mắt ở bài
vẫy tay). Nguyên tắc tổng quát: quỹ đạo tham chiếu phải liên tục đến ít nhất
đạo hàm bậc 1 (thường bậc 2 — acceleration) → dẫn đến spline, trajectory
optimization.
*Liên quan: `week3_squat_wave.py` — tay (sin) giật nhẹ lúc đầu, chân (1−cos) êm.*

### 3.3. Quy tắc −θ/+2θ/−θ cho squat hoạt động vì sao?
**Trả lời:** Góc nghiêng cộng dồn từ thân xuống bàn chân: (−θ)+(2θ)+(−θ)=0
→ bàn chân luôn song song với thân → chân phẳng trên sàn, thân thẳng đứng,
hông hạ thấp. Đây là ràng buộc kinematic dạng đóng đơn giản nhất; trường
hợp tổng quát cần IK.

---

## 4. Kinematics & IK (tuần 4)

### 4.1. Jacobian là gì? Dùng để làm gì?
**Trả lời:** Ma trận J sao cho ẋ = J·q̇ — ánh xạ vận tốc khớp sang vận tốc
điểm cuối (end-effector). Dùng cho: IK vi phân, biến đổi lực (τ = Jᵀ·F),
phát hiện singularity (J mất hạng), phân tích tầm với.

### 4.2. Vì sao không giải IK bằng nghịch đảo Jacobian trực tiếp J⁻¹?
**Trả lời:** (1) J thường không vuông (vd 3×7 với tay 7 DOF) nên không có
nghịch đảo; (2) gần singularity (tay duỗi thẳng hết), J gần mất hạng →
nghịch đảo/pseudo-inverse cho Δq nổ tung. Giải pháp: damped least squares
Δq = Jᵀ(JJᵀ + λ²I)⁻¹·err — λ đánh đổi chính xác lấy ổn định gần singularity.
*Liên quan: `week4_ik_reach.py`.*

### 4.3. Integrator windup là gì? Gặp ở đâu và fix thế nào?
**Trả lời:** Khi mục tiêu không thể đạt được (ngoài tầm với / khớp chạm giới
hạn), sai số không về 0 → biến tích lũy (q_des, hoặc khâu I của PID) bị cộng
dồn vô hạn. Triệu chứng thực tế: q_des khớp elbow lên 17 rad (~1000°) trong
khi khớp thật kẹt ở giới hạn → khi sai số đổi chiều, hệ mất kiểm soát.
Fix: clamp biến tích lũy trong giới hạn vật lý (anti-windup).
*Liên quan: bug thật đã debug ở tuần 4.*

### 4.4. Time-scale separation trong điều khiển phân tầng là gì?
**Trả lời:** Vòng ngoài (IK, planner) phải chậm hơn đáng kể vòng trong (PD
khớp) để vòng trong kịp bám. Vi phạm → vòng ngoài ra lệnh nhảy quá nhanh,
vòng trong bám trễ, vọt lố qua lại thành dao động. Bug thực tế: cho phép
mục tiêu khớp đổi 10 rad/s → tay rung 25 rad/s và quật ngã cả robot; fix
bằng giới hạn tốc độ nhích mục tiêu xuống 1.5 rad/s.
*Liên quan: `week4_ik_reach.py` — hằng số MAX_STEP.*

### 4.5. Tay 7 DOF với tới điểm 3D — thừa 4 DOF. Redundancy được xử lý thế nào?
**Trả lời:** Vô số tư thế khớp cho cùng vị trí tay (khuỷu chĩa xuống/ngang).
DLS mặc định chọn nghiệm "ít di chuyển nhất" từ tư thế hiện tại (minimum
norm). Chủ động hơn: chiếu mục tiêu phụ (tránh giới hạn khớp, tư thế tự
nhiên) vào null space của J — nền tảng của whole-body control.

### 4.6. Quy trình debug một hệ điều khiển bị mất ổn định?
**Trả lời:** (1) Cô lập: đơn giản hóa đầu vào (mục tiêu động → tĩnh) để tách
nguyên nhân; (2) Đo lường: log biến trạng thái theo thời gian (chiều cao,
tốc độ khớp, sai số) tìm thời điểm hỏng; (3) Đối chiếu con số với giới hạn
vật lý (25 rad/s ở khớp tay là phi lý → dao động); (4) Fix một thứ một lần,
retest sau mỗi fix. Không đoán mò, không sửa nhiều thứ cùng lúc.

### 4.7. Windup có thể tái xuất ở đâu ngoài PID/IK cơ bản?
**Trả lời:** Bất cứ đâu có một biến được CỘNG DỒN mỗi vòng lặp dựa trên một
"khoảng cách còn thiếu". Gặp lại y hệt khi implement null-space projection
(mục tiêu phụ cho khuỷu tay dao động trong lúc IK giữ tay cố định): lần đầu
cho vận tốc null-space đổ trực tiếp vào bộ tích lũy q_des rồi clip theo
MAX_STEP — bị kẹp trần suốt nửa chu kỳ dao động → tích lũy thành đường dốc
khổng lồ → khuỷu tay lệch ra ngoài ý muốn, sai số tay vọt lên 140mm (thay
vì đứng yên). Fix: đổi mục tiêu phụ từ "vận tốc tùy ý" thành "một tư thế
phụ cụ thể dao động theo thời gian", rồi CHỈ kéo về nó bằng một tỷ lệ nhỏ
(kiểu P-controller) — sau fix sai số tay giảm về 5mm. Bài học: nhận diện
lại được windup nhanh ở một ngữ cảnh hoàn toàn khác (null space thay vì IK
thô) là dấu hiệu hiểu bản chất, không chỉ nhớ định nghĩa.
*Liên quan: `week4_nullspace.py`.*

---

## 5. Chủ đề sắp học (điền dần khi đến bài)

### 5.1. LQR là gì? Q và R đóng vai trò gì?
**Trả lời:** LQR tính RA công thức điều khiển tối ưu `u = -Kx` từ mô hình
tuyến tính (A, B) của hệ thống, thay vì đoán hệ số như PD. Q = "mức độ
ghét từng loại lỗi trạng thái" (đường chéo, mỗi số ứng với 1 biến), R =
"mức độ ghét tốn lực điều khiển". Đây là một phép cân bằng (trade-off):
Q lớn → phản ứng mạnh, bám sát nhưng tốn lực; R lớn → tiết kiệm lực
nhưng bám chậm. Bằng chứng số cụ thể: đặt Q cho góc gậy = 10.0 (lớn nhất
trong 4 biến) → gain K tương ứng của góc gậy ra 47.91 — lớn nhất trong 4
hệ số K. Tăng Q(theta) lên 100 thì K(theta) tăng theo tương ứng — chứng
minh trực tiếp "khai báo ghét gì → nhận phản ứng mạnh đúng ở đó", không
cần đoán hệ số bằng tay như PD.
*Liên quan: `week5_lqr_cartpole.py` — con lắc ngược trên xe.*

### 5.2. Vì sao LQR chỉ hoạt động GẦN điểm cân bằng, không phải mọi góc?
**Trả lời:** A, B được suy ra bằng cách TUYẾN TÍNH HÓA phương trình động
lực học phi tuyến quanh điểm cân bằng (gậy thẳng đứng, θ=0) — dùng xấp xỉ
sin(θ)≈θ, cos(θ)≈1, vốn chỉ đúng khi θ nhỏ. Thí nghiệm thực tế: đẩy gậy
lệch ~8.6° thì LQR bắt lại êm trong ~2s; đẩy lệch quá ~30-35° thì xấp xỉ
sai lệch quá nhiều so với vật lý thật (phi tuyến), xe phản ứng sai hướng/
sai độ lớn, gậy ngã hẳn dù công thức K không đổi. Đây là lý do humanoid
control thực tế cần MPC hoặc RL (không cần tuyến tính hóa, xử lý được
toàn dải góc) khi robot lệch xa khỏi tư thế chuẩn.

### 5.3. Capture point là gì? Vì sao nó = vị trí + vận tốc/ω₀?
**Trả lời:** Capture point là điểm DUY NHẤT trên sàn mà đặt chân vào đó sẽ
dừng ngã ngay lập tức, trong mô hình LIPM (con lắc ngược với chiều cao
khối tâm không đổi). Trực giác: giống hứng cây chổi đang đổ trên lòng bàn
tay — bị đẩy mạnh (vận tốc lớn) thì tay/chân phải di chuyển xa hơn mới
đuổi kịp. Công thức `x_cp = x + ẋ/ω₀` chỉ là "vị trí hiện tại + một đoạn
tỉ lệ với vận tốc"; `ω₀ = √(g/z_com)` là tần số tự nhiên của cơ thể — gập
gối thấp (giảm z_com) làm ω₀ tăng, capture point gần hơn, dễ giữ thăng
bằng hơn (lý do võ sĩ đứng tấn thấp). Tính trực tiếp từ `data.subtree_com`
của G1 trong MuJoCo cho ra đúng hướng vật lý khi bị đẩy các hướng khác nhau.
*Liên quan: `week6_capture_point.py`.*

### 5.4. Capturability region là gì? Vì sao có cú đẩy "không ai cứu được"?
**Trả lời:** Chân có giới hạn chiều dài bước (MAX_STEP) và thời gian mỗi
bước (T_STEP) — nếu capture point nằm xa hơn khả năng bước, phải bước lại
nhiều lần, mỗi lần tính lại capture point mới (giống bị xô mạnh phải chạy
dồn vài bước mới vững). Nhưng có một NGƯỠNG vận tốc đẩy cứng: vượt ngưỡng
đó, vận tốc/độ lệch tăng dần qua mỗi bước dù bước đúng cách và kịch hết
sải chân — không chuỗi bước nào cứu được, bất kể thuật toán. Đo thực tế
bằng dò nhị phân: với MAX_STEP=0.28m, T_STEP=0.30s, ngưỡng là ~0.505 m/s.
Đây là giới hạn THIẾT KẾ CƠ KHÍ (chân dài hơn/động cơ nhanh hơn → ngưỡng
cao hơn), không phải giới hạn thuật toán — MPC/RL cũng không phá được nó.
*Liên quan: `week7_lipm_steps.py`.*

### 5.5. MPC khác gì capture-point-greedy? Có "cứu" được các cú đẩy vượt ngưỡng không?
**Trả lời:** KHÔNG — đây là điểm dễ hiểu lầm nhất. MPC tối ưu ĐỒNG THỜI
nhiều bước tương lai (thay vì mỗi bước chỉ nhắm capture point rồi clip),
nhưng không thể vượt ngưỡng capturability vật lý (5.4) — đó là giới hạn
cứng, không phải do thiếu tầm nhìn. Giá trị thật của MPC nằm ở HIỆU QUẢ
trong vùng còn cứu được: đo thực nghiệm, ở v=0.503m/s greedy cần 8 bước
để dừng hẳn, MPC (nhìn trước 2 bước, tối ưu bằng scipy) chỉ cần 6 bước;
ở v=0.505 (sát ngưỡng) là 10 vs 8 bước. Ngoài ngưỡng ~0.507m/s, CẢ HAI đều
ngã — con số giống nhau đến từng mm, xác nhận ngưỡng là vật lý thật.
**Bài học debug quan trọng:** lần đầu so sánh, MPC "thắng" một cách đáng
ngờ (cứu được cả những cú đẩy vượt ngưỡng) — hóa ra do lỗi mô hình: MPC
lỡ giả định chân "dịch chuyển tức thời" trước khi ngã, còn greedy đúng là
ngã dưới chân trụ CŨ suốt cả bước rồi mới đổi chân ở cuối. Hai bên dùng
vật lý khác nhau nên so sánh không công bằng. Sửa cho cùng giả định vật
lý, kết quả "đẹp nhưng sai" biến mất, thay bằng kết quả khiêm tốn hơn
nhưng ĐÚNG (ít bước hơn, không vượt ngưỡng). Bài học tổng quát: một kết
quả benchmark "quá tốt để tin" thường là do hai phương án đang được so
sánh trên hai giả định khác nhau, không phải do phương án mới thực sự
giỏi hơn — luôn kiểm tra cả hai baseline dùng CHUNG một mô hình trước khi
tin kết luận.
*Liên quan: so sánh `greedy_recover` vs `mpc_horizon_recover`, tuần MPC.*

### 5.6. Whole-Body Control (WBC) là gì? Vai trò của trọng số và W_REG?
**Trả lời:** WBC giải NHIỀU mục tiêu Cartesian đồng thời (ví dụ 2 tay với
2 vật khác nhau) trong MỘT bài toán QP, thay vì null-space cứng nhắc
(1 ưu tiên chính + 1 phụ, tuần 4) — đây là "soft priority": mọi mục tiêu
cùng tham gia, CÂN BẰNG bằng trọng số W_i (đúng vai trò Q trong LQR —
"ghét lỗi nào nhiều hơn"). Công thức: minimize Σ ||W_i(J_i·dq - v_i)||²,
subject to -DQ_MAX ≤ dq ≤ DQ_MAX (giới hạn đưa vào NGAY TỪ ĐẦU, không
clip sau — tinh thần MPC). W_REG (trọng số nhỏ phạt ||dq||²) là "phần
thưởng cho đứng yên nếu không cần thiết" — xử lý các khớp dư (redundancy,
tuần 4) không ảnh hưởng mục tiêu chính, và chống nổ số gần singularity
(vai trò tương đương λ²I trong DLS). Bằng chứng số: tăng W_R từ 3.0 lên
10.0 (tay phải ưu tiên hơn) → tay phải bám mục tiêu chính xác hơn rõ
rệt, tay trái "nhường" nhiều hơn khi 2 mục tiêu tranh nhau qua khớp vai
chung — quan sát trực tiếp trên robot, không chỉ suy luận trên giấy.
*Liên quan: `week9_wbc.py`.*

### 5.7. WBC dùng để "giữ thăng bằng qua thân trên" thất bại — tại sao, và đó có phải bug không?
**Trả lời:** KHÔNG phải bug — là giới hạn thật của phương pháp. Thử điều
khiển CoM (khối tâm toàn robot) bằng cách di chuyển thân trên+tay (kiểu
squat+vẫy tay tuần 3, chủ động dùng phản lực để bù), trong khi chân vẫn
giữ PD tĩnh (tuần 2) không đồng bộ — robot ngã dù công thức QP hoàn toàn
đúng về toán. Nguyên nhân: đây là điều khiển ở CẤP ĐỘ ĐỘNG HỌC (chỉ
Jacobian, biến=vận tốc khớp) — KHÔNG có động lực học (khối lượng, mô-men
quán tính) và KHÔNG có ràng buộc lực tiếp xúc chân-sàn (chân chỉ đẩy
được, không hút được — nhắc lại bài floating base/underactuated). Thân
trên xoay đủ mạnh để "giúp" CoM về đúng vị trí theo TOÁN, nhưng tạo phản
lực thật mà chân tĩnh không biết để chống lại → đổ. Đây chính là lý do
WBC sản xuất thật (Boston Dynamics, Unitree) luôn giải QP với ĐẦY ĐỦ
động lực học (M(q)q̈+C=τ+Jᵀf) VÀ ràng buộc tiếp xúc, không chỉ Jacobian
động học đơn thuần. Fix thực tế: thu hẹp phạm vi — bỏ khớp eo/thân khỏi
WBC, chỉ còn 2 tay (không ảnh hưởng thăng bằng chân), đứng vững 12s.
**Bài học debug đi kèm** (2 lần sai trước khi tới lỗi này): (1) đo CoM
bằng `subtree_com[0]` (toàn robot) nhưng tính Jacobian bằng
`mj_jacSubtreeCom(...,torso_id)` (chỉ subtree con torso) — hai đại lượng
lệch định nghĩa, giống lỗi so sánh sai mô hình ở MPC (mục 5.5); (2) dùng
`err/DT` làm vận tốc mục tiêu = đòi sửa hết lỗi trong 1 chu kỳ 2ms (gain
≈500) → QP luôn kịch giới hạn theo hướng nhiễu, sửa bằng gain P vừa phải.
*Liên quan: `week9_wbc.py` — phần "BA LAN DEBUG THAT BAI" trong docstring.*

### 5.8. RL: PPO, reward design, domain randomization, sim-to-real — TODO tháng 5–7

---

## 6. Dựng mô phỏng & cầu nối ROS2 (Isaac Sim + G1)

Phần này toàn lỗi thực tế đã tự gặp và tự sửa khi dựng mô phỏng G1 với
LiDAR 3D + camera, phát sang ROS 2 Jazzy cho SLAM. Kể được các câu chuyện
này có sức nặng hơn nhiều so với thuộc lý thuyết.

### 6.1. Kể một lỗi mà việc SỬA một thứ lại phá thứ khác.
**Trả lời:** GPU RTX 5070 Ti (Blackwell, sm_120) cần PyTorch bản cu128;
bản Isaac Sim cài kèm là cu126 nên báo "no kernel image available". Chạy
`pip install --force-reinstall torch` để sửa — nhưng lệnh đó kéo theo
numpy 2.4.4 đè lên numpy 1.26.0 mà Isaac Sim ghim. Hậu quả: **mọi cảm
biến** (cả camera lẫn LiDAR) crash với thông báo hoàn toàn không liên
quan: `TypeError: Unable to write from unknown dtype, kind=f, size=0`.
Bài học: lỗi hiện ra ở nơi rất xa nguyên nhân; khi một thay đổi nhỏ làm
hỏng thứ tưởng không liên quan, hãy nghi ngờ dây chuyền phụ thuộc trước.

### 6.2. Kể một lỗi "im lặng" — không có exception nhưng hệ thống sai.
**Trả lời:** Ba ví dụ khác nhau:
- `LidarRtx(translation=[0,0,H])` gán translation cho CẢ prim cha lẫn prim
  con `sensor` → cảm biến thật nằm ở độ cao **2H**. Point cloud báo mặt
  sàn ở z = −2H thay vì −H. Nguy hiểm vì bản đồ vẫn "trông có vẻ đúng",
  chỉ sai tỷ lệ độ cao. Phát hiện bằng cách đặt 2 LiDAR ở 2 độ cao khác
  nhau và đo tỷ lệ → ra −2.01 ở CẢ HAI → khẳng định lỗi hệ thống.
- `IsaacComputeOdometry` với `chassisPrim="/World/G1"` (sai — articulation
  root thật ở `/World/G1/pelvis`): node **chết lặng**, `/odom` không được
  phát, không có exception. Chỉ thấy khi chủ động đọc log
  `omni.graph.core.plugin`.
- `ROS2RtxLidarHelper` mặc định `fullScan=False` → phát từng mảnh quét
  (~5k điểm @91Hz) thay vì nguyên vòng (~27k @10Hz). Vẫn có dữ liệu nên
  dễ bỏ qua, nhưng SLAM 3D mong đợi vòng quét hoàn chỉnh.

### 6.3. Bạn debug thế nào khi dữ liệu "trông đúng" nhưng nghi ngờ có sai?
**Trả lời:** Đối chiếu cùng một dữ liệu qua HAI hệ tọa độ khác nhau để
khoanh vùng. Ví dụ thật: point cloud tính trong hệ `pelvis` cho 88% điểm
đúng mặt sàn, nhưng qua hệ `odom` chỉ còn 10%. Kết luận ngay: cảm biến
không sai, mà **tư thế robot báo cáo qua odometry** sai. Truy tiếp ra
robot bị nghiêng 3° — do vòng lặp là *đặt tư thế → chạy vật lý 1/60s →
đọc odometry*, và trong 1/60s đó trọng lực kéo hai chân lủng lẳng làm
nghiêng hông. 3° nghe nhỏ nhưng ở khoảng cách 20m lệch hơn 1m. Fix: tắt
trọng lực cho robot khi nó đang ở chế độ trượt.

### 6.4. Vì sao point cloud của camera có thể ĐÚNG trong khi ảnh RGB bị nghiêng?
**Trả lời:** Hai thứ phụ thuộc hai điều kiện khác nhau. Point cloud đi
kèm frame TF: nếu dữ liệu và TF nghiêng CÙNG một kiểu thì hai cái triệt
tiêu nhau, RViz đặt điểm đúng vị trí thật. Ảnh RGB chỉ là lưới pixel,
không có gì bù trừ → camera nghiêng bao nhiêu thì ảnh nghiêng bấy nhiêu.
Hệ quả thực tế: với SLAM chỉ cần point cloud đúng, ảnh nghiêng vô hại.
Nhưng nếu xoay camera cho ảnh thẳng thì phá vỡ sự triệt tiêu đó → phải
chỉnh frame theo. Lưu ý bổ sung: xoay 90° cũng làm **hình dạng vùng quét**
đổi từ rộng-ngang sang cao-dọc (khung hình chữ nhật 640×480), nên "đúng"
ở đây chỉ có nghĩa "mỗi điểm rơi đúng bề mặt thật", không phải "quét đúng
vùng mong muốn".

### 6.5. Lỗi phép ĐO khác lỗi hệ thống thế nào? Cho ví dụ đã gặp.
**Trả lời:** Đây là bài học lặp lại 3 lần trong cùng một buổi:
- **Thước đo quá thô:** dùng độ sáng trung bình 4 vùng ảnh để tìm góc
  camera. Hai tư thế KHÁC HẲN nhau (nhìn trái vs nhìn sau) cho ra con số
  GIỐNG HỆT (trái 161 / phải 228), vì cảnh đối xứng nên bầu trời đều
  chiếm một nửa khung. Từ đó kết luận sai rằng "tham số không có tác dụng".
- **Tiêu chí thiếu ràng buộc:** dò ma trận xoay cho frame camera bằng
  tiêu chí "% điểm nằm trên mặt sàn". BỐN ma trận khác nhau đều cho 77.5%
  — chúng chỉ khác nhau ở phương vị (trước/sau/trái/phải), mà mặt sàn thì
  ở khắp nơi nên tiêu chí không phân biệt được. Kết quả: chọn nhầm ma
  trận làm camera quét sang TRÁI thay vì phía trước. Chỉ phát hiện khi
  người dùng nhìn RViz thấy camera và LiDAR không chồng khớp. Fix: thêm
  điều kiện thứ hai (tâm vùng quét phải ở phía trước) → lọc ra đúng một
  ma trận. Đo lại: khoảng cách camera→LiDAR gần nhất giảm từ 43.7cm
  xuống 11.6cm.
- **Bản tái hiện không trung thực:** dựng preset để tái hiện trạng thái
  lỗi cũ nhưng lỡ gán frame của phiên bản khác vào → đo ra 0.1% và kết
  luận sai rằng dữ liệu cũ hỏng, trong khi thực tế nó đúng (13.4%).

**Bài học chung:** khi kết quả đo mâu thuẫn với quan sát trực tiếp, nghi
ngờ PHÉP ĐO trước khi nghi ngờ hệ thống. Và trước khi tin một tiêu chí
tối ưu, hãy kiểm tra xem có nhiều nghiệm cùng điểm số hay không.

### 6.6. Vì sao Isaac Sim và ROS2 không import trực tiếp lẫn nhau được?
**Trả lời:** Isaac Sim 5.0 chạy Python 3.11 (conda env), ROS 2 Jazzy trên
Ubuntu 24.04 chạy Python 3.12 — `rclpy` biên dịch cho 3.12 nên không
import được vào 3.11. Giải pháp: Isaac Sim phát topic bằng thư viện ROS2
**C++ nội bộ** (extension ship sẵn cả `humble/` và `jazzy/`), còn
Nav2/SLAM/RViz chạy tiến trình riêng bằng Python hệ thống. Hai bên gặp
nhau ở tầng DDS. Kèm theo: mọi node ROS2 làm việc với mô phỏng phải bật
`use_sim_time` và mô phỏng phải phát `/clock` — nếu không, RViz coi mọi
dữ liệu là "đến từ quá khứ" (timestamp mô phỏng bắt đầu từ 0 còn hệ thống
dùng epoch) và vứt bỏ hết, ngập tràn cảnh báo `TF_OLD_DATA`.

### 6.7. Kể một lỗi mà sửa cái này lại làm LỘ RA cái khác.
**Trả lời:** Camera trải qua ba lớp lỗi xếp chồng:
1. Camera đặt ở z=+0.50 nằm gọn trong `torso_link` (link này chiếm z từ
   +0.03 đến +0.53, đo bằng `UsdGeom.BBoxCache`) → khung hình là khối đen.
2. Dời camera ra ngoài (z=+0.62) → lộ ra ảnh bị **nghiêng 90°**, do
   `rep.create.camera` tự gắn sẵn phép xoay `(90,0,90)` vào prim con mà
   `rep.modify.pose` chỉ tác động lên prim cha → hai phép xoay cộng dồn.
3. Sửa góc xoay → lộ ra **nửa khung hình đen** vì cảnh chỉ có một
   `SphereLight` chiếu một phía; thêm `DomeLight` mới sáng đều.

Chừng nào khung hình còn đen thì không thể phát hiện góc nghiêng, và
chừng nào ảnh còn nghiêng thì không đánh giá được độ sáng có đều không.
Bài học: "sửa xong lại hỏng" nhiều khi là dấu hiệu **tiến bộ** — mỗi lần
sửa vén được một lớp.

---

## Mẹo phỏng vấn

- Mỗi câu trả lời nên có 3 tầng: **định nghĩa 1 câu → trực giác vật lý →
  ví dụ/bug thực tế mình từng gặp**. Tầng 3 là thứ khiến bạn khác ứng viên
  chỉ học lý thuyết — và khóa học này đang tích lũy chính tầng đó.
- Khi bị hỏi thứ không biết: nói thẳng không biết, rồi suy luận to thành
  tiếng từ nguyên lý gần nhất mình có. Interviewer robotics đánh giá cách
  suy nghĩ hơn đáp án.
- Chuẩn bị kể trơn tru 2–3 câu chuyện debug (windup, oscillation, actuator
  interface) theo cấu trúc: triệu chứng → cách cô lập → số liệu → fix.
