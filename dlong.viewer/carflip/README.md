# Car Flip Recovery — mô phỏng xe lật trên không và vòi phun khí lạnh

Mô phỏng một chiếc SUV điện ~2 tấn lao khỏi mặt đường và lật trên không. IMU trên xe phát hiện rơi tự do và ước lượng tư thế; 12 vòi phun khí lạnh (giả định) xoay xe về tư thế bánh xuống trước khi chạm đất.

**Chạy:** mở `index.html` bằng trình duyệt (Chrome hoặc Edge). Chạy offline, không cần cài gì.
**Đo số liệu:** `node bench.js 100` in bảng kết quả và ghi file CSV. Số liệu tái lập được theo seed.

## Kịch bản (chọn theo cơ chế lật có thật)

| Kịch bản | Cơ chế | Vai trò khi demo |
|---|---|---|
| Lao qua gờ đất mép taluy | Mất lái lệch phải ψ, bánh phải leo lên gờ đất ở mép đường đắp cao, xe lăn và bay xuống ruộng thấp hơn D mét. Thời gian bay ~1–1,5 s. | Kịch bản chính, có thời gian để cứu |
| Trượt ngang vào lề đất mềm | Lật do vấp (tripped rollover), dạng lật phổ biến nhất: bánh cày vào đất, lật ngay tại chỗ, bay ~0,3 s. | Ca lỗi vật lý: vòi phun không kịp |
| Bị húc hông | Xung lực vào hông sau bên trái, xe xoay và trượt ra mép đường đắp. | Đầu vào khác để giám khảo thử |

Giám khảo có thể kéo thanh trượt (tốc độ, góc lệch, độ cao gờ, độ cao taluy...) hoặc bấm **Kịch bản ngẫu nhiên** (phím R).

## Hệ thống

- **Vật lý** (`core.js`): vật rắn 6 bậc tự do, bước 1 ms (bán ẩn Euler). Tư thế dùng quaternion. Bốn bánh có lò xo và giảm chấn kèm ma sát lốp; 18 điểm thân xe dùng tiếp xúc lò xo phạt (penalty) với mặt đất.
- **Cảm biến 100 Hz:**
  - Gyro: thêm độ lệch (bias) và nhiễu.
  - Gia tốc kế: đo lực riêng (specific force), có nhiễu, bão hoà ở ±16 g.
  - GNSS 10 Hz.
- **Phát hiện rơi tự do:** |f| < 0,4 g liên tục trong 30 ms.
- **Ước lượng tư thế** (bộ lọc bù kiểu Mahony): gyro tích phân liên tục. Gia tốc kế chỉ được dùng để sửa độ nghiêng khi |f| ≈ 1 g và xe không bay (**có cổng**).
- **Điều khiển 100 Hz** (`baselineController`, thay được qua `opts.controller`):
  - Bộ PD trên vector quay về tư thế đứng, phân bổ bật/tắt cho 12 vòi phun.
  - Mũi xe được canh theo hướng vận tốc để tiếp đất lăn bánh, không trượt ngang.
  - Phanh ngược ở 0,6 s cuối.
  - Van có độ trễ 20 ms, khí nén có hạn (Isp 70 s).

## Trụ cột Track 4 trong hệ thống

| Trụ cột | Nằm ở đâu |
|---|---|
| Biểu diễn | Trạng thái [p, v, q, ω]. Dùng quaternion vì góc Euler bị khoá (gimbal lock) khi xe chúc 90°. Kết quả tiếp đất được mã hoá thành an toàn / cứng / lật. |
| Hình học | Hệ toạ độ thân xe và thế giới. Gia tốc kế đo f = Rᵀ(a − g). Mô-men vòi phun τ = r × F. |
| Bất định | Gyro có độ lệch và nhiễu. Cổng tin cậy cho gia tốc kế: khi rơi tự do nó đọc ≈ 0 g nên không dùng được làm "hướng lên". |
| Thời gian | IMU và điều khiển 100 Hz, GNSS 10 Hz, độ trễ van, độ trễ phát hiện rơi tự do, thời gian còn lại trước khi chạm đất. |
| Dữ liệu | Log CSV 100 Hz cho mỗi lần chạy. Chạy lại tất định theo seed. |
| Bằng chứng | Monte Carlo so sánh theo cặp, khoảng tin cậy Wilson, ca lỗi có nguyên nhân. |

## Số đo (node bench.js 100, seed 1–100)

| Cấu hình | Tiếp đất an toàn | KTC 95% (Wilson) | Cứu được* | Sai số tư thế lúc chạm |
|---|---|---|---|---|
| Không vòi phun | 7/100 | 3–14% | – | – |
| Vòi phun + ước lượng có cổng | **86/100** | 78–91% | 81/93 | 3,1° |
| Vòi phun + bộ lọc ngây thơ (ca lỗi) | 72/100 | 63–80% | 66/93 | 11,1° |
| Vòi phun + tư thế thật (cận trên) | 91/100 | 84–95% | 84/93 | 0° |

\* Trong 93 kịch bản mà xe không vòi phun không tiếp đất an toàn, cấu hình đó cứu được bao nhiêu.

"An toàn" nghĩa là: tiếp đất bằng bánh, nghiêng ≤ 25°, vận tốc rơi ≤ 9 m/s (tương đương rơi tự do ~4 m), nóc không chạm đất.

## Ca lỗi

1. **Bộ lọc ngây thơ tin gia tốc kế khi đang bay.** Khi rơi tự do, gia tốc kế chỉ đo được nhiễu; khi vòi phun bật, nó đo lực đẩy. Bộ lọc coi các số đo đó là "hướng lên", nên tư thế ước lượng trôi sai (sai số lúc chạm đất 11° so với 3°), và tỉ lệ an toàn giảm từ 86% xuống 72%.
   - Sửa: dùng cổng tin cậy (chỉ dùng gia tốc kế khi |f| ≈ 1 g và xe không bay).
   - Xem trực tiếp: chọn **Ngây thơ** và bật **Khung tư thế xe tự ước lượng**.
2. **Lật do vấp** (kịch bản "trượt ngang"). Xe chỉ bay ~0,3–0,5 s, lật tiếp trên mặt đất, và vòi phun hết khí mà không cứu được. Đây là giới hạn vật lý, không phải lỗi phần mềm: mô-men cần thiết lớn hơn 10–20 lần mức vòi phun cung cấp.
3. Có vài ca xe tiếp đất đứng bánh rồi trượt ngang trên mái taluy và lật tiếp. Bộ điều khiển chỉ lo đến lúc chạm đất.

## Giới hạn (những gì chưa chứng minh)

- Thông số xe (khối lượng, quán tính, lò xo) và vòi phun là **giả định**, không phải thông số đã công bố của hãng nào.
- Tiếp xúc dùng lò xo phạt: thân xe không biến dạng, lốp đơn giản hoá. Gia tốc đỉnh (g) chỉ để so sánh tương đối.
- Monte Carlo chỉ chạy trên kịch bản "gờ taluy" trong dải tham số ở `RANGES`. Kết quả không nói gì về các dạng lật khác.
- Vòi phun khí lạnh trên ô tô là ý tưởng khái niệm. Phần dùng được ngay là **phát hiện rơi tự do và lật từ IMU**, vì đó là đầu vào để kích hoạt túi khí rèm và eCall.
