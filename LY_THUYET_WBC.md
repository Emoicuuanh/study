# Lý thuyết nền cho WBC trên G1 — Phần 1

Viết cho người biết lập trình, quen ROS và phần cứng, nhưng chưa học động lực học
vật rắn và tối ưu hoá. Mọi con số trong tài liệu là **số thật của con G1 trong
phòng**, không phải ví dụ bịa.

Phần 1 này dựng nền và dẫn tới **một phương trình duy nhất**. Gần như mọi thứ
trong dự án đều mọc ra từ nó.

---

## 0. Cái gì cần học, cái gì không

Nhiều tài liệu về humanoid mở đầu bằng một bức tường ký hiệu. Phần lớn không cần
cho việc ta đang làm.

**Cần:**
- Vectơ và ma trận: nhân, nghịch đảo, "hệ phương trình có bao nhiêu nghiệm"
- Đạo hàm theo thời gian: vị trí → vận tốc → gia tốc
- Định luật 2 Newton, và dạng của nó cho vật quay
- Ý tưởng "tìm giá trị nhỏ nhất của một hàm, với vài điều kiện bắt buộc"

**Chưa cần (mình sẽ nói khi nào cần):**
- Hình học vi phân, nhóm Lie, `SO(3)`, `SE(3)` — đẹp nhưng chưa cần để hiểu
- Lagrange, Hamilton — cách *suy ra* phương trình; ta chỉ cần *dùng* nó
- Lý thuyết điều khiển tối ưu, MPC, học tăng cường

Quy tắc đọc: **nếu một ký hiệu không giúp bạn đoán được con số sẽ ra sao, bỏ qua
nó.**

---

## 1. Robot là gì, về mặt toán học

### 1.1. Khớp và bậc tự do

G1 có **29 khớp** quay. Mỗi khớp một con số: góc hiện tại. Gom lại thành một vectơ
29 phần tử. Trong code là `q_motor`.

Thứ tự khớp (theo chỉ số của Unitree, ta dùng y nguyên):

```
 0- 5  chân trái    (hông pitch/roll/yaw, gối, cổ chân pitch/roll)
 6-11  chân phải    (thứ tự y hệt)
12-14  eo           (yaw, roll, pitch)
15-21  tay trái     (vai pitch/roll/yaw, khuỷu, cổ tay roll/pitch/yaw)
22-28  tay phải
```

Phép thử hôm trước cho khuỷu tay trái co lên chính là kiểm tra bảng này: ta ra
lệnh cho **khớp 18**, và đúng khớp 18 cử động. Nếu bảng sai, một khớp khác đã cử
động — và robot đang treo nên sai cũng vô hại. Đó là lý do phép thử đó tồn tại.

### 1.2. Thân nổi — chỗ khác biệt quan trọng nhất

Một cánh tay robot công nghiệp **bắt vít xuống sàn**. Biết 6 góc khớp là biết
chính xác đầu kẹp ở đâu.

Robot chân thì không. Nó **không gắn vào gì cả**. Biết cả 29 góc khớp vẫn không
biết robot đang đứng ở đâu, hay đang nghiêng thế nào — nó có thể đang đứng, đang
nằm, đang bay.

Nên để mô tả đầy đủ, cần thêm **6 con số nữa**: vị trí của thân (3) và hướng của
thân (3). Gọi là **thân nổi** (floating base).

```
tổng cộng:  6  (thân nổi)  +  29  (khớp)  =  35 bậc tự do
```

Con số **35** đó chính là `nv` xuất hiện khắp trong code. (Có chỗ thấy 36 — vì
hướng trong không gian được lưu bằng 4 số quaternion thay vì 3, để tránh một vấn
đề số học. Vận tốc vẫn là 35.)

**Vì sao điều này quan trọng đến thế:** robot có 35 bậc tự do nhưng chỉ có **29
động cơ**. Sáu bậc tự do của thân nổi **không có động cơ nào**. Bạn không thể ra
lệnh cho robot "hãy dịch sang phải" — không có động cơ nào nối thân với thế giới.

Robot chỉ có thể dịch chuyển bằng cách **đẩy vào mặt đất**.

Nghe hiển nhiên, nhưng viết nó ra thành phương trình chính là cả bài toán thăng
bằng. Ta sẽ đến đó ở mục 3.

### 1.3. Động học thuận

Cho 35 con số (vị trí/hướng thân + 29 góc khớp), tính ra vị trí và hướng của *mọi*
khâu trên robot: bàn chân ở đâu, khuỷu tay ở đâu, lidar hướng nào.

Đây là phép nhân ma trận lan từ gốc ra ngọn. Thư viện làm hết (ta dùng
**Pinocchio**). Điều cần nhớ: **nó chỉ là hình học, không có vật lý nào** — không
khối lượng, không lực.

### 1.4. Jacobian — từ "khớp quay" ra "điểm chạy"

Đây là khái niệm cần nắm chắc, vì nó xuất hiện ở mọi chỗ.

Câu hỏi: *nếu các khớp đang quay với vận tốc này, thì mũi bàn chân đang chạy với
vận tốc nào?*

Quan hệ đó **tuyến tính** (với vận tốc, không phải với vị trí). Nên viết được
thành phép nhân ma trận:

```
vận tốc của điểm  =  J  ×  vận tốc các khớp
```

`J` là **Jacobian** của điểm đó. Mỗi điểm trên robot có Jacobian riêng, và nó thay
đổi theo tư thế hiện tại.

Hình dung: `J` là **bảng tỉ số truyền**. Cột thứ `k` trả lời *"nếu chỉ riêng khớp
`k` quay 1 rad/s, điểm này chạy theo hướng nào, nhanh bao nhiêu?"*

Hai tính chất dùng liên tục:

**Chiều ngược — lực.** Cùng ma trận đó, chuyển vị, cho biết: *nếu có lực `f` tác
dụng vào điểm, nó tạo ra mô-men bao nhiêu ở mỗi khớp?*

```
mô-men tại các khớp  =  Jᵀ  ×  lực tại điểm
```

Đây là lý do `Jᵀf` xuất hiện trong mọi phương trình phía sau. Cầm tạ ở tay thì
khuỷu phải gồng — `Jᵀ` chính là cái tính "gồng bao nhiêu".

#### `Jᵀ` nghĩa là gì, và vì sao lại đúng là chuyển vị

**Về mặt máy móc**, chuyển vị chỉ là lật hàng thành cột:

```
J  =  [ 2  5  1 ]          Jᵀ  =  [ 2  7 ]
      [ 7  3  4 ]                 [ 5  3 ]
                                  [ 1  4 ]
     (2 hàng × 3 cột)            (3 hàng × 2 cột)
```

Kích thước đảo ngược — đó chính là lý do nó hợp để đi theo chiều ngược lại. (Với
một con số đơn lẻ thì lật chẳng đổi gì, nên ở con lắc ta sẽ không thấy khác biệt.)

**Vì sao lại đúng là chuyển vị** thì bạn đã biết nguyên lý rồi, chỉ chưa viết
thành công thức: **đòn bẩy**.

Đòn bẩy tỉ lệ 2:1 — ấn đầu ngắn 1 cm thì đầu dài đi 2 cm (*được quãng đường gấp
đôi*), nhưng nâng 10 N ở đầu dài thì phải ấn 20 N ở đầu ngắn (*mất lực gấp đôi*).
Hộp số, ròng rọc đều vậy: **được tốc độ thì mất lực, theo đúng cùng một tỉ số.**

Lý do là công suất không tự sinh ra. Viết ra:

```
cong suat o cac khop  =  cong suat o diem tiep xuc
       τ · q̇          =          f · v
```

Thay `v = J·q̇` (định nghĩa Jacobian):

```
τ · q̇  =  f · (J·q̇)
```

Đẳng thức này phải đúng **với mọi** `q̇` — bất kể khớp quay kiểu gì. Chỉ một cách
duy nhất để điều đó xảy ra:

```
τ  =  Jᵀ · f
```

**Chuyển vị xuất hiện chính vì năng lượng không tự sinh ra.** Không phải quy ước
toán học, mà là hệ quả vật lý.

**Điểm kỳ dị.** Có những tư thế mà `J` "mất hạng" — tức tồn tại một hướng mà dù
khớp quay kiểu gì, điểm cũng không đi theo hướng đó được.

Trên G1, chỗ này quan trọng đến mức đáng dừng lại giải thích kỹ. Và bạn thử được
ngay trên chính mình.

#### Thử bằng cơ thể bạn

Đứng dậy, **khoá thẳng đầu gối**. Để ý đùi: cơ gần như không phải làm gì. Trọng
lượng đi thẳng từ hông xuống mắt cá **qua xương**, không qua cơ.

Giờ **chùng gối xuống 15–20°** rồi giữ nguyên. Đùi gồng ngay, và đứng lâu kiểu đó
là mỏi.

Cùng một trọng lượng. Nhưng một tư thế thì cơ không chịu lực, tư thế kia thì chịu.

#### Vì sao: cánh tay đòn

Gối chỉ sinh mô-men khi lực đi **lệch khỏi tâm quay** của nó. Khoảng lệch đó là
cánh tay đòn.

- **Gối thẳng:** hông, gối, mắt cá thẳng hàng. Lực xuyên qua tâm gối. Cánh tay
  đòn bằng 0 → mô-men bằng 0.
- **Gối chùng 0.3 rad:** gối nhô ra trước ~4–5 cm. Với 190 N đè lên một chân,
  mô-men gối ≈ **8–9 Nm**. (Đo thật trên robot: **−13.8 Nm**.)

#### Chỗ này mới là vấn đề

G1 **không có cảm biến lực dưới bàn chân**. Ta phải suy ngược lực từ mô-men khớp:

> *"gối đang gồng 13.8 Nm, cánh tay đòn 4.5 cm → lực đè lên chân cỡ 190 N."*

Thử đúng logic đó với gối thẳng:

> *"gối đang gồng 0 Nm, cánh tay đòn 0 cm → lực đè lên chân là... bao nhiêu cũng
> được."*

**Mô-men khớp không còn mang thông tin gì về lực.** Dù robot chịu 0 N hay 400 N,
gối vẫn báo 0 Nm. Phép tính ngược mất căn cứ. Đó chính là ý nghĩa của "`J` mất
hạng": có một *hướng lực* mà khớp hoàn toàn không cảm nhận được.

#### Số điều kiện là gì

Bỏ cái tên đi một lúc, và quay lại ý "bảng tỉ số truyền" ở trên. Điểm mấu chốt:
**tỉ số truyền khác nhau theo hướng.**

Ma trận ta đang nói biến *lực ở bàn chân* thành *mô-men ở khớp*. Ấn vào bàn chân
100 N, mỗi hướng cho kết quả khác hẳn:

| đẩy bàn chân 100 N theo hướng | mô-men sinh ra ở gối |
|---|---|
| **ngang** (ra trước/sau) | ~50 Nm |
| **dọc theo xương chân** (gối thẳng) | ~0.00005 Nm |

Tỉ số giữa hai con số đó **chính là số điều kiện**: hướng tốt nhất chia cho hướng
tệ nhất, ở đây là `50 / 0.00005 = 10⁶`.

#### Vì sao đọc ngược lại thì nổ tung

Ta cần đi ngược: đo mô-men, suy ra lực — tức **chia cho tỉ số truyền**. Với nhiễu
đo `±0.3 Nm`:

```
huong ngang     : do 50 Nm ± 0.3  ->  luc = 100 N ± 0.6 N       tot
huong doc xuong : do  0 Nm ± 0.3  ->  luc =   0 N ± 600000 N    vo nghia
```

Nhiễu bị chia cho một số cực nhỏ nên phình ra khổng lồ.

Ví dụ bằng số thuần, nếu thấy dễ hình dung hơn. Một cái hộp nhận hai núm vặn và
cho ra hai kim chỉ:

```
kim 1 = 1000  × a      (num a rat nhay)
kim 2 = 0.001 × b      (num b gan nhu khong nhuc nhich kim)
```

Số điều kiện = `1000 / 0.001` = một triệu. Làm ngược — nhìn kim, đoán núm:

```
a = kim1 / 1000     kim1 lech 0.3 -> a lech 0.0003    tot
b = kim2 / 0.001    kim2 lech 0.3 -> b lech 300       tham hoa
```

Cùng một sai số đọc kim. Nhưng một bên **chia** cho 1000, bên kia **nhân** với
1000.

> **Số điều kiện lớn = ma trận có hướng "điếc". Đi xuôi thì không sao, đi ngược
> thì nhiễu nổ tung.**

#### Con số 99 N từ đâu ra

Mô-men đo được trên robot này có nhiễu khoảng **0.3 Nm**:

| | tín hiệu | nhiễu | tỉ lệ | sai số lực |
|---|---|---|---|---|
| gối thẳng | ~0 Nm | 0.3 Nm | vô nghĩa | **99 N** |
| gối chùng 0.3 rad | ~8.5 Nm | 0.3 Nm | 28 lần | **1.4 N** |

Cùng một đoạn mã, cùng một robot, chỉ khác tư thế — sai số lệch nhau **70 lần**.

Nên "luôn để robot đứng gối chùng" không phải mẹo vặt, mà là điều kiện để phép đo
**có nghĩa**. Nếu lúc nào đó thấy ước lượng lực nhảy loạn, câu hỏi đầu tiên là
*gối có đang duỗi thẳng không*. Đây là một trong hai giới hạn ghi ngay đầu file
`leg_odometry.py`.

---

## 2. Động lực học — thêm khối lượng vào

Mục 1 chỉ có hình học. Giờ thêm vật lý.

### 2.1. Phương trình đến từ đâu — dẫn từ con lắc

Phương trình ở mục sau **không phải định luật mới**. Nó chính là `F = ma`, viết
bằng góc khớp thay vì bằng toạ độ x-y-z. Dẫn ra bằng ví dụ đơn giản nhất.

**Bài toán.** Thanh dài `L = 0.5 m`, đầu thanh gắn khối lượng `m = 1 kg`, gốc
thanh là khớp quay có động cơ. Góc là `q`.

**Bước 1 — Jacobian.** Khối lượng chạy trên cung tròn bán kính `L`:

```
v = L · q̇        ->  J = L = 0.5
a = L · q̈
```

**Bước 2 — Newton, rồi quy về khớp.** Lực cần để tạo gia tốc đó:

```
F = m · a = m · L · q̈
```

Nhưng ta không đẩy trực tiếp vào khối lượng — ta vặn ở khớp. Lực cách khớp `L`
thì cần mô-men `L × F`:

```
τ = L · (m · L · q̈) = m·L² · q̈
```

**Bước 3 — đó chính là `M`.** So với dạng `M·q̈ = τ`, ta thấy `M = m·L² = 0.25`.

Để ý `M` **không phải** khối lượng — nó là `m` nhân `L` **bình phương**. Và `L`
chính là Jacobian. Chữ `L` xuất hiện hai lần vì **một lần đi xuôi** (khớp quay →
khối lượng chạy) và **một lần đi ngược** (lực ở khối lượng → mô-men ở khớp).

Ở robot thật, "đi xuôi" là `J`, "đi ngược" là `Jᵀ`:

```
M  =  Jᵀ · m · J
```

Đọc **từ phải sang trái**, nó kể đúng ba bước vừa làm:

```
          q̈       gia toc cac khop
    J  ·  q̈       ->  gia toc khoi luong        (di xuoi)
m · J  ·  q̈       ->  luc can de tao gia toc    (F = ma)
Jᵀ· m · J · q̈     ->  mo-men can o cac khop     (di nguoc)
```

*(Kiểm kích thước cho chắc: `J` là `3×n`, `Jᵀ` là `n×3`, nên `Jᵀ·m·J` ra `n×n` —
đúng cỡ của `M`. Viết `J·m·J` thì thậm chí không nhân được.)*

**Bước 4 — thêm trọng lực, ra `h`.** Trọng lực kéo khối lượng xuống `m·g`, sinh
mô-men ở khớp phụ thuộc góc:

```
m·L²·q̈  +  m·g·L·sin(q)  =  τ
   └ M ┘      └─── h ───┘
```

Ở `q = 90°` (thanh nằm ngang): `1 × 9.81 × 0.5 = 4.9 Nm` — động cơ phải giữ 4.9 Nm
chỉ để thanh không rơi. Hợp lý.

*(Phần `q̇` trong `h(q,q̇)` là hiệu ứng ly tâm/Coriolis. Nó sinh ra vì `J` đổi theo
tư thế: đạo hàm `v = J·q̇` ra `a = J·q̈ + J̇·q̇`, và cái `J̇·q̇` chui vào `h`. Con lắc
đơn giản này chưa có nó.)*

**Bước 5 — thêm ngoại lực.** Ai đó đẩy vào khối lượng lực `f`, nó cũng sinh mô-men
ở khớp theo quy tắc cũ:

```
m·L²·q̈  +  m·g·L·sin(q)  =  τ  +  Jᵀ·f
```

**Xong.** Đó là phương trình đầy đủ, cho trường hợp một khớp.

**Từ con lắc lên G1** — ba thay đổi, không có gì mới về bản chất:

| con lắc | G1 |
|---|---|
| 1 khối lượng | ~30 khâu → cộng đóng góp của tất cả: `M = Σ Jᵢᵀ·mᵢ·Jᵢ` |
| 1 khớp → `M` là một số | 35 bậc tự do → `M` là ma trận 35×35 |
| gốc thanh gắn vào tường | **thân nổi** → thêm 6 toạ độ không có động cơ |

Các phần tử **ngoài đường chéo** của `M` sinh ra ở phép cộng đó: một khâu thường
chịu ảnh hưởng của *nhiều* khớp cùng lúc (cẳng tay phụ thuộc cả vai lẫn khuỷu).
Đó là lý do vật lý của câu "các khớp ảnh hưởng lẫn nhau".

Thay đổi thứ ba là chỗ sinh ra toàn bộ bài toán thăng bằng. Với con lắc, gốc gắn
vào tường nên **tường** cho phản lực bao nhiêu cũng được. Robot thì không có tường.

### 2.2. Từ `F = ma` đến robot

Với một chất điểm: `F = ma`. Với robot 35 bậc tự do, dạng tương đương là:

```
M(q) · q̈  +  h(q, q̇)  =  τ  +  Jᵀ f
```

Đây là **phương trình trung tâm**. Mọi thứ trong dự án đều mọc ra từ nó. Đọc từng
số hạng:

**`q̈` — gia tốc của 35 bậc tự do.** Thứ ta muốn tạo ra. Muốn robot đang đổ về
trước dừng lại thì phải tạo gia tốc ngược chiều.

**`M(q)` — ma trận quán tính, 35×35.** Vai trò giống chữ `m` trong `F = ma`, nhưng
là ma trận vì robot có nhiều phần nối nhau. Ngoài đường chéo khác 0 nghĩa là
**các khớp ảnh hưởng lẫn nhau**: vung tay thì thân xoay theo. `(q)` nghĩa là nó
đổi theo tư thế — tay duỗi và tay co cho quán tính khác nhau.

**`h(q, q̇)` — trọng lực cộng hiệu ứng quay.** Gom hai thứ: mô-men cần để chống
trọng lực, và các lực kiểu ly tâm/Coriolis sinh ra khi robot đang chuyển động. Đây
là "thứ xảy ra nếu không làm gì cả".

**`τ` — mô-men động cơ.** Thứ ta ra lệnh. **29 con số, không phải 35.**

**`Jᵀ f` — lực từ mặt đất, quy về mô-men khớp.** `f` là lực mặt đất đẩy vào bàn
chân; `Jᵀ` biến nó thành mô-men ở từng khớp, đúng như mục 1.4.

Đọc cả câu bằng lời thường:

> *Quán tính nhân gia tốc, cộng với trọng lực và hiệu ứng quay, bằng mô-men động
> cơ cộng với tác dụng của lực mặt đất.*

### 2.3. Sáu hàng đầu tiên — chìa khoá của toàn bộ bài toán

Phương trình trên có 35 hàng. Hãy xem riêng **6 hàng đầu**, tức phần ứng với thân
nổi.

Như đã nói ở mục 1.2: **thân nổi không có động cơ nào.** Nên ở 6 hàng đó, `τ`
bằng 0:

```
M[0:6] · q̈  +  h[0:6]  =  Jᵀ[0:6] · f
```

Trong code (`wbc.py`) chính là dòng này:

```python
Ceq1 = np.zeros((6, nz)); Ceq1[:, :nv] = M[:6]; Ceq1[:, nv:] = -JcT[:6]
beq1 = -h[:6]
```

**Đây là toàn bộ bài toán thăng bằng, cô đọng trong sáu dòng.** Nó nói:

> *Chỉ có lực từ mặt đất mới đổi được chuyển động tổng thể của robot. Động cơ
> không làm được điều đó.*

Hệ quả là mọi thứ về sau:

- Muốn robot ngừng đổ, không thể "ra lệnh cho nó đứng thẳng". Phải **sắp xếp lực
  dưới chân** cho đúng.
- Mà lực dưới chân thì bị giới hạn (mục tiếp theo). Nên có những cú đẩy **không
  thể nào** cứu được bằng cách đứng yên — phải bước chân.
- Và mô-men động cơ trở thành *hệ quả*, không phải *mục tiêu*: ta chọn lực mong
  muốn, rồi mô-men là thứ cần thiết để tạo ra lực đó.

Nếu chỉ nhớ một điều từ tài liệu này, nhớ điều này.

### 2.4. Vì sao thân nổi lại thành 6 ràng buộc chứ không phải 6 biến tự do

Một câu dễ nhầm. Sáu bậc tự do thân nổi *không* phải thứ ta điều khiển, nhưng cũng
*không* phải thứ ta bỏ qua. Chúng xuất hiện trong bài toán dưới dạng **ràng buộc
bắt buộc**: bất kỳ bộ `(q̈, f)` nào ta chọn cũng **phải** thoả 6 phương trình
trên, nếu không thì nó mô tả một chuyển động vi phạm định luật Newton.

Nên trong QP, 6 hàng đó nằm ở phần "đẳng thức" — không thương lượng được, khác
với các mục tiêu (giữ trọng tâm, giữ tư thế) chỉ là "cố gắng đạt được".

---

## 3. Bài tập tự kiểm

Không cần tính toán, chỉ cần trả lời bằng lời. Nếu trả lời được thì phần 1 đã
ngấm.

1. Vì sao biết đủ 29 góc khớp vẫn không biết robot đang ở đâu?
2. Robot đang treo trên giàn, chân không chạm đất. Theo phương trình ở mục 2.3,
   nó có thể tự làm thân mình dịch sang ngang được không? Vì sao?
3. Vì sao ta luôn để robot đứng gối chùng khi đo?
4. Nếu mô hình khối lượng sai (ví dụ quên cục router 2 kg sau lưng), số hạng nào
   trong phương trình sai, và hậu quả là gì?
5. Câu khó: đứng **hai chân**, có *nhiều* cách chia lực giữa hai bàn chân mà vẫn
   cho cùng một hợp lực. Điều đó có mâu thuẫn với 6 phương trình ở mục 2.3 không?

*(Câu 5 chính là thứ đã làm mô-men hông của ta cao gấp 14 lần Unitree. Phần 2 sẽ
giải thích.)*

---

## 4. Đáp án

Tự trả lời trước rồi hãy đọc.

**1. Vì sao 29 góc khớp vẫn chưa đủ?**

Vì chúng chỉ mô tả **hình dạng**, không mô tả **chỗ nằm trong thế giới**. Cùng bộ
29 góc có thể là robot đang đứng, đang nằm sấp, hay đang rơi giữa không trung.

Và điều đó đẻ ra vấn đề thực tế: robot **không có cảm biến đo trực tiếp 6 số đó**.
Cảm biến quán tính cho *hướng* (3 trong 6); *vị trí* thì không có gì đo cả. Đó là
lý do phải viết leg odometry, và lý do ta **không bao giờ biết vị trí tuyệt đối** —
chỉ biết vận tốc. Nên WBC phải **neo vào bàn chân** thay vì dùng toạ độ tuyệt đối.
(Lần đầu quên neo, robot ngã sau 3.8 s trong mô phỏng dù mọi đại lượng động lực
học đều khớp đến 1e-5.)

**2. Robot treo lơ lửng có tự dịch ngang được không?**

**Không.** Chân không chạm đất → `f = 0` → `M[0:6]·q̈ + h[0:6] = 0`. Không còn
ngoại lực nào ngoài trọng lực, nên trọng tâm chỉ **rơi** được, không trôi ngang.

**Nhưng nó xoay được** — mèo rơi từ cao lật người giữa không trung bằng đúng cách
đó: không ngoại lực, mô-men động lượng tổng vẫn bằng 0, mà thân vẫn đổi hướng nhờ
cử động các khớp.

*(Chi tiết thực tế: trong thí nghiệm của ta `f` **không** bằng 0 — dây giàn là một
ngoại lực. Nhưng ta không điều khiển được nó, nên kết luận không đổi.)*

Đó chính là lý do giàn treo an toàn: không phải vì phần mềm đúng, mà vì ở trạng
thái đó **vật lý không cho phép robot đi đâu cả**, kể cả khi phần mềm sai hoàn toàn.

**3. Vì sao luôn để gối chùng khi đo?**

Gối thẳng thì cánh tay đòn bằng 0 → mô-men khớp bằng 0 **bất kể lực lớn bao nhiêu**.
Mà robot không có cảm biến lực dưới bàn chân, nên mô-men khớp là nguồn thông tin
*duy nhất* về lực.

```
goi thang     : tin hieu ~0 Nm,   nhieu 0.3 Nm  ->  sai so luc  99 N
goi chung 0.3 : tin hieu ~8.5 Nm, nhieu 0.3 Nm  ->  sai so luc  1.4 N
```

**4. Quên cục router 2 kg thì sao?**

Sai **`M` và `h`** — cả hai đều dựng từ mô hình khối lượng. `h` sai rõ nhất: thiếu
`2 × 9.81 = 19.6 N` trọng lực, cộng mô-men do nó nằm lệch sau lưng.

Hậu quả: WBC tính cho robot 36.6 kg trong khi robot thật 38.6 kg → ra lệnh đẩy
xuống đất thiếu 19.6 N → robot **từ từ trụt xuống**, và tác vụ trọng tâm phải gồng
bù. Tệ hơn là **vị trí trọng tâm lệch**, nên cái đích WBC nhắm tới cũng sai →
robot đứng nghiêng có hệ thống.

Đó là lý do ta đã cân và hiệu chuẩn cục đó, rồi kiểm chứng độc lập bằng tâm áp
lực: khối tâm mô hình khớp số đo đến **1.2 mm**.

**5. Nhiều cách chia lực có mâu thuẫn với 6 phương trình không?**

**Không. Và đây là chỗ quan trọng nhất.** Đếm thử. Robot đứng yên (`q̈ = 0`):

```
h[0:6]  =  Jᵀ[0:6] · f

so an         : 8 diem tiep xuc × 3 thanh phan luc  =  24
so phuong trinh                                     =   6
                                        con lai  =  18 chieu tu do
```

Sáu phương trình chỉ ràng buộc **tổng** lực và **tổng** mô-men. Chúng hoàn toàn
không nói gì về **cách chia**.

Nghĩa là hai bàn chân có thể **ép vào nhau**: chân trái đẩy trước 15 N, chân phải
đẩy sau 15 N — tổng bằng 0 nên 6 phương trình không hề thấy, nhưng mô-men khớp thì
khác hẳn. Đó là **lực nội tại**, và nó chính là thủ phạm:

| | của ta | Unitree |
|---|---|---|
| mô-men hông trung bình | **28.7 Nm** | 1.9 Nm |
| tổng 12 khớp chân | 126.3 Nm | 53.3 Nm |
| tương quan với sai lệch trọng tâm | **−0.08** | +0.51 |

Con số cuối lộ ra bản chất: mô-men hông của ta **không hề liên quan** đến việc giữ
thăng bằng — nó là lực nội tại vô nghĩa, hai chân ghì nhau cho vui. QP chọn bừa
một điểm trong không gian 18 chiều đó, vì hàm mục tiêu cũ **không hề phạt mô-men**.

Sửa bằng cách thêm số hạng phạt mô-men (`w_tau`). Đo trên robot:

```
mo-men hong:  28.7  ->  4.3 Nm     (Unitree 2.6)
tong 12 khop: 126.3 ->  70.0       (Unitree 59.2)
tuong quan:   -0.08 -> +0.615      (Unitree +0.428)
```

Và **sai lệch trọng tâm không xấu đi chút nào** — vì ta chỉ chọn điểm khác trong
18 chiều tự do, không đụng vào 6 phương trình bắt buộc.

Đó là ý nghĩa thật của **vô định tĩnh học**: bài toán có vô số lời giải đúng, và
nếu không nói rõ mình muốn gì thì máy tính chọn bừa một cái.

---

## Phần 2 sẽ có gì

- **Thăng bằng thật sự là gì**: trọng tâm, tâm áp lực, đa giác đỡ. Vì sao "giữ
  thăng bằng" = "giữ tâm áp lực trong lòng bàn chân", và vì sao điều đó có giới
  hạn cứng.
- **Điểm capture**: công thức nói trước cú đẩy mạnh bao nhiêu thì *bắt buộc* phải
  bước chân. Ta đã dùng nó để kiểm chứng bản mô phỏng.
- **Tiếp xúc**: nón ma sát, và vì sao mặt đất "chỉ đẩy chứ không kéo" lại là một
  ràng buộc bất đẳng thức.
- **Vô định tĩnh học**: đáp án câu 5.
