# Lý thuyết nền cho WBC trên G1 — Phần 2: Thăng bằng

Tiếp theo [phần 1](LY_THUYET_WBC.md). Phần 1 kết thúc ở sáu hàng:

```
M[0:6]·q̈  +  h[0:6]  =  Jᵀ[0:6]·f
```

*Chỉ lực mặt đất mới đổi được chuyển động tổng thể.* Phần 2 trả lời câu tiếp theo:
**lực mặt đất làm được những gì, và không làm được những gì.**

---

## 1. Trọng tâm

Robot có ~30 khâu, mỗi khâu một khối lượng. **Trọng tâm** là điểm mà toàn bộ khối
lượng có thể coi như tụ về đó, khi xét chuyển động tổng thể.

Trên G1 đứng gối chùng:

```
khoi luong  = 38.6 kg   ->  trong luong = 38.6 × 9.81 = 378.7 N
chieu cao trong tam ≈ 0.689 m
```

Trọng tâm **không phải một điểm cố định trên robot**. Giơ tay lên, nó dịch lên và
ra trước. Gập gối, nó tụt xuống. Mỗi chu kỳ ta phải tính lại.

Vì sao nó quan trọng: ba hàng đầu trong sáu hàng kia, dịch ra lời thường, chính là

```
38.6 kg  ×  (gia toc trong tam)  =  tong ngoai luc
```

Nên **điều khiển thăng bằng = điều khiển gia tốc của trọng tâm**. Mà muốn thế thì
phải điều khiển tổng ngoại lực. Mà ngoại lực chỉ đến từ mặt đất.

---

## 2. Mặt đất chỉ đẩy được, không kéo được

Nghe hiển nhiên đến mức dễ bỏ qua. Nhưng nó là **ràng buộc cứng nhất** của cả bài
toán.

Chân robot không dính xuống sàn. Mặt đất có thể **đẩy lên** bao nhiêu cũng được,
nhưng **không kéo xuống được một chút nào**. Viết thành công thức, cho mỗi điểm
tiếp xúc:

```
fz  ≥  0
```

Đây là **bất đẳng thức**, không phải đẳng thức. Và đó là lý do bài toán phải giải
bằng tối ưu hoá chứ không giải thẳng ra được bằng đại số.

Trong code ta để `fz ≥ 1.0 N` thay vì `≥ 0` — chặt hơn một chút, để tránh trường
hợp biên "lực đúng bằng 0" gây rắc rối cho bộ giải.

### Hệ quả: tâm áp lực

Mỗi bàn chân G1 có **4 điểm tiếp xúc** (lấy từ mô hình MJCF của Unitree):

```
x = -0.05 ... +0.12 m      (5 cm sau mat ca  ->  12 cm truoc mat ca)
y = ±0.025 ... ±0.030 m
```

Tức bàn chân dài **17 cm**, rộng khoảng **6 cm**.

Mặt đất đẩy lên ở cả bốn điểm, mỗi điểm một lực khác nhau. Gộp lại, ta có thể thay
cả bốn bằng **một lực duy nhất đặt tại một điểm** — điểm đó gọi là **tâm áp lực**
(CoP).

Vì mọi `fz` đều `≥ 0`, tâm áp lực là một **trung bình có trọng số không âm** của
bốn điểm. Và trung bình có trọng số không âm thì **luôn nằm trong bao lồi** của
các điểm đó.

> **Tâm áp lực không bao giờ ra khỏi vùng bàn chân chạm đất.**
> Không phải vì bộ điều khiển kém, mà vì mặt đất không kéo được.

Vùng đó gọi là **đa giác đỡ**. Đứng hai chân thì nó là bao lồi của cả tám điểm —
gồm cả khoảng trống giữa hai bàn chân.

### Thử bằng cơ thể bạn

Đứng thẳng, nghiêng người ra trước **thật chậm**. Cảm nhận áp lực dưới bàn chân
dịch dần về phía ngón. Đó chính là tâm áp lực đang chạy ra trước.

Nghiêng thêm nữa. Đến một lúc, áp lực dồn hết vào mũi chân — tâm áp lực **chạm mép
đa giác đỡ**. Nghiêng thêm một chút nữa thì **bắt buộc phải bước**, không có cách
nào khác.

Không phải bạn yếu. Là vật lý hết chỗ.

---

## 3. Điều kiện đứng yên

Robot đứng **hoàn toàn yên** (`q̈ = 0`, `q̇ = 0`). Sáu hàng thành:

```
h[0:6]  =  Jᵀ[0:6] · f
```

Dịch ra: tổng lực mặt đất phải cân bằng trọng lực, **và** tổng mô-men phải bằng 0.
Điều kiện mô-men bằng 0 chính là:

> **Tâm áp lực phải nằm ngay dưới trọng tâm.**

Ghép với mục 2:

> **Đứng được ⟺ hình chiếu của trọng tâm xuống đất nằm trong đa giác đỡ.**

Đó là toàn bộ lý thuyết thăng bằng **tĩnh**. Đơn giản đến mức đáng ngờ — và đúng
là chưa đủ, vì robot thật không bao giờ đứng hoàn toàn yên.

---

## 4. Khi robot đang chuyển động — con lắc ngược

Nếu trọng tâm đang **trôi** thì hình chiếu nằm trong đa giác đỡ vẫn chưa chắc cứu
được: nó đang chạy ra ngoài.

Để xử lý, ta cần một mô hình đơn giản hoá. Mô hình tiêu chuẩn là **con lắc ngược
tuyến tính** (LIPM — Linear Inverted Pendulum Model):

- Coi toàn bộ robot như **một khối lượng điểm** ở độ cao `h`, gắn với mặt đất bằng
  một thanh không khối lượng
- Giữ `h` **không đổi** (robot không nhún lên xuống)

Với giả thiết đó, chuyển động ngang của trọng tâm tuân theo:

```
ẍ  =  ω² · (x − x_cop)        voi   ω = √(g/h)
```

Đọc bằng lời: *trọng tâm càng lệch xa tâm áp lực thì càng bị đẩy ra xa thêm.* Dấu
**dương** — đó là lý do gọi là con lắc **ngược**: nó tự khuếch đại, không tự về.

Với G1 đứng gối chùng:

```
ω = √(9.81 / 0.689) = 3.77 rad/s
```

Con số `ω` này sẽ xuất hiện ở mọi tính toán phía sau. Nó là **nhịp tự nhiên của
cú ngã** — robot càng thấp thì `ω` càng lớn, ngã càng nhanh. (Trẻ con ngã nhanh
hơn người lớn, cùng lý do.)

---

## 5. Điểm capture — câu trả lời gọn cho "có cứu được không"

Đây là khái niệm đẹp nhất trong mảng này.

Câu hỏi: *trọng tâm đang ở `x`, đang trôi với vận tốc `v`. Phải đặt tâm áp lực ở
đâu để robot dừng hẳn lại?*

Giải phương trình LIPM ra đáp án **một dòng**:

```
ξ  =  x  +  v / ω
```

`ξ` gọi là **điểm capture**. Ý nghĩa: *nếu đặt tâm áp lực đúng tại `ξ`, robot sẽ
chậm dần và dừng hẳn.* Đặt ra trước điểm đó thì robot bị đẩy ngược lại; đặt sau
thì nó tiếp tục ngã.

Và vì tâm áp lực **không ra khỏi đa giác đỡ được** (mục 2), ta có ngay tiêu chí:

> **`ξ` nằm trong đa giác đỡ → cứu được mà không cần bước.**
> **`ξ` nằm ngoài → bắt buộc phải bước chân, không có lựa chọn nào khác.**

Toàn bộ trạng thái thăng bằng rút gọn thành **một điểm trên mặt đất**. Nhìn điểm
đó so với bàn chân là biết ngay tình hình.

### Tính thử cho G1

Cú đẩy `F` kéo dài `T` giây truyền cho robot một xung lượng:

```
xung luong = F · T       ->   v = F · T / m
```

Trong mô phỏng ta đẩy `T = 0.15 s` về phía sau. Mép sau đa giác đỡ cách mặt cá
`0.05 m`, và trọng tâm đứng hơi chếch ra trước `0.014 m`, nên biên còn lại là
`0.064 m`.

```
v_max  =  0.064 × 3.77  =  0.241 m/s
F_max  =  m · v_max / T  =  38.6 × 0.241 / 0.15  =  62 N
```

**Lý thuyết nói: đẩy quá ~62 N thì phải bước chân.**

Đo thật trong MuJoCo: **qua được 62 N, hỏng ở 65 N.**

Sai lệch ~5%, và lệch về phía **tốt hơn dự đoán**. Lý do: LIPM giả thiết robot là
một khối lượng điểm, nên nó **bỏ qua việc vung tay và xoay thân**. Robot thật dùng
được cả hai để mua thêm chút biên — đúng như bạn quơ tay khi suýt ngã. Trong WBC
của ta, đó chính là tác vụ điều tiết mô-men động lượng.

Đây là một kiểm chứng tốt: **một mô hình đơn giản, tính bằng tay trên giấy, dự
đoán đúng đến 5% một con số đo được từ mô phỏng đầy đủ 35 bậc tự do.**

---

## 6. Nón ma sát — ràng buộc thứ hai

Mặt đất đẩy lên được bao nhiêu cũng được. Nhưng đẩy **ngang** thì không.

Chân đặt trên sàn trơn, lực ngang quá lớn thì **trượt**. Giới hạn là định luật ma
sát quen thuộc:

```
√(fx² + fy²)  ≤  μ · fz
```

Vẽ ra, tập hợp các lực hợp lệ là một hình **nón** dựng đứng từ điểm tiếp xúc. Nên
gọi là **nón ma sát**.

Ta dùng `μ = 0.6`. Nghĩa là mỗi 100 N đè xuống thì đẩy ngang được tối đa 60 N.

Trong code, thay vì dùng hình nón tròn (khó cho bộ giải), ta xấp xỉ bằng **hình
chóp**: bốn bất đẳng thức tuyến tính thay cho một bất đẳng thức bậc hai.

```
|fx| ≤ μ·fz        ->  2 bat dang thuc
|fy| ≤ μ·fz        ->  2 bat dang thuc
```

Chặt hơn hình nón thật một chút ở các góc — phía an toàn, nên chấp nhận được. Đổi
lại, bài toán giữ được dạng **tuyến tính**, và điều đó quyết định việc giải được
trong 2 mili-giây hay không.

### Hệ quả ít ai để ý

Nón ma sát + `fz ≥ 0` cộng lại **tự động** ép tâm áp lực nằm trong đa giác đỡ. Ta
không phải viết thêm ràng buộc nào cho tâm áp lực cả — nó là hệ quả.

Đó là lý do trong `wbc.py` bạn sẽ **không tìm thấy** dòng nào tên là "CoP". Nó
được đảm bảo bởi tám điểm tiếp xúc, mỗi điểm chỉ đẩy được chứ không kéo.

---

## 7. Tóm lại: thăng bằng là gì

Gộp cả phần 1 và phần 2:

| | |
|---|---|
| **Muốn gì** | điều khiển gia tốc trọng tâm |
| **Làm được bằng gì** | chỉ lực mặt đất *(6 hàng đầu, phần 1)* |
| **Lực đó bị giới hạn** | chỉ đẩy, không kéo · trong nón ma sát · chỉ ở chỗ chân chạm đất |
| **Hệ quả** | tâm áp lực không ra khỏi đa giác đỡ |
| **Nên** | có cú đẩy không thể cứu bằng cách đứng yên |
| **Biết khi nào** | điểm capture `ξ = x + v/ω` nằm trong hay ngoài đa giác đỡ |

---

## 8. Bài tập tự kiểm

1. Vì sao robot gập gối thấp xuống thì chống đẩy tốt hơn? *(Gợi ý: `ω = √(g/h)`.)*
   Và cái giá phải trả là gì?
2. Đứng **một chân**, đa giác đỡ nhỏ lại còn một bàn chân. Theo lý thuyết trên,
   ngưỡng đẩy chịu được thay đổi thế nào?
3. Sàn trơn (`μ = 0.2` thay vì 0.6). Ràng buộc nào bị siết, và robot mất khả năng
   gì?
4. Vì sao ta xấp xỉ nón ma sát bằng hình chóp, dù biết nó chặt hơn thực tế?
5. Câu khó: điểm capture nằm **ngay trên mép** đa giác đỡ. Robot đang ở ranh giới
   cứu được / không cứu được. Nhưng trong thực tế ta **không nên** để nó tới đó.
   Vì sao? *(Gợi ý: nhớ lại nhiễu đo vận tốc thân — 7 mm/s.)*

---

## Phần 3 sẽ có gì

- **Tối ưu hoá**: vì sao bài toán lại thành "tìm bộ số tốt nhất thoả các điều cấm"
- **QP là gì**, và vì sao đúng dạng đó mới giải kịp trong 2 ms
- **Trọng số trong hàm mục tiêu** = thứ tự ưu tiên, và chuyện gì xảy ra khi đặt sai
- Rồi phần 4: đọc thẳng vào `wbc.py`, từng dòng nối với từng khái niệm đã học
