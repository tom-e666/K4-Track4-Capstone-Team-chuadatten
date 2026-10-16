# JET3 · TowerFlight Lab

Bản độc lập cho xe phản lực xuyên ba tháp ở ba góc ngã tư. Toàn bộ mã chạy, thư viện Three.js, dữ liệu, script và kiểm thử nằm trong thư mục này. Có thể sao chép riêng cả thư mục để chạy; không cần npm install, Python hay đọc file từ thư mục cha.

Yêu cầu: Node.js 18 trở lên và trình duyệt có WebGL.

Từ thư mục `JET3-TowerFlight-Lab`, chạy:

```powershell
node scripts/serve.cjs
```

Mở http://127.0.0.1:8766/#jetlab. Có thể chọn cổng khác bằng `node scripts/serve.cjs 8787`. Cũng có thể mở trực tiếp `viewer/index.html#jetlab` để xem mẫu có sẵn.

- `Lab phản lực`: hai thanh trượt lực nâng tổng 4 vòi (20–56 kN) và tốc độ tối đa đặt (18–144 km/h). Thả thanh trượt để tích phân lại ca đang chọn; khi chưa chọn ca thì chạy cấu hình danh định.
- Chọn một điểm hoặc ca lưu để phát lại dữ liệu gốc và khôi phục policy của file. Bảng hiển thị tốc độ thực tế, mức vượt tốc độ đặt và mã quỹ đạo đã đối chiếu với script.
- Sau khi đổi giới hạn, kết quả chỉ áp dụng cho ca đang xem. Biểu đồ giữ dữ liệu gốc. Bấm `Xuất policy.json` để dùng giới hạn mới cho lần sinh dữ liệu tiếp theo.
- Chọn hiệu ứng `Khói trắng`, `Khí sáng` hoặc `Tắt hiệu ứng`. Khói được tăng kích thước 5 lần, phát từ bốn cụm ở bánh xe và trôi theo lực/gió.
- `Đủ thân tháp` là mặc định. `Cắt lớp quan sát` cho phép nhìn rõ tầng xe chạy khi cần.

Sinh lại dữ liệu bằng uniform sampling và CEM:

```powershell
node scripts/generate-jet-scenarios.cjs --seed 20261016 --sample-js viewer/jet-lab-sample.js
node tests/test_jet_sim.cjs evidence/jet-stunt/viewer.json
```

Mặc định: 600 mẫu ban đầu và 6 vòng × 500 mẫu = 3.600 lần thử. Script ghi `evidence/jet-stunt/viewer.json`, `summary.json`, `cases.json`. Nạp `viewer.json` trong giao diện để xem dữ liệu khác. Tái tạo toàn bộ biểu đồ với policy đã tải về:

```powershell
node scripts/generate-jet-scenarios.cjs --policy "C:\duong-dan\jet-policy.json" --out evidence/custom --sample-js viewer/jet-lab-sample.js
```

`viewer/jet-sim.js` là lõi dùng chung cho Node và trình duyệt. Bộ điều khiển PD nhận trạng thái thật (vị trí/vận tốc XYZ, yaw và tốc độ quay), quỹ đạo tham chiếu cùng các giới hạn. Đầu ra là 4 vector lực `(Fx, Fy, Fz)` cho 4 vòi. Bộ mô phỏng tích phân lực, trọng lực, lực cản và nhiễu gió. Camera, radar và LiDAR chưa tham gia vòng điều khiển phản lực.

Đây là mô hình tịnh tiến 3D và yaw động; pitch/roll chỉ để hiển thị. Va chạm dùng kiểm tra cắt mặt tường và bao hình xe, chưa phải mô phỏng rigid-body 6-DoF. Tốc độ được hãm bằng lực có giới hạn; nhiễu và độ trễ có thể gây vượt tốc tạm thời. CEM chủ động tìm ca lỗi nên tỷ lệ chấm đỏ không phải xác suất tai nạn thực tế.

Các chương đường phố cũ được giữ trong bản viewer sao chép để tham khảo. Những lệnh Python được nhắc trong các chương đó thuộc dự án gốc; tính năng xe phản lực dùng các lệnh Node ở trên.

Thư viện Three.js được sao chép từ viewer gốc, giữ nguyên thông báo bản quyền trong `viewer/vendor/three.min.js`.
