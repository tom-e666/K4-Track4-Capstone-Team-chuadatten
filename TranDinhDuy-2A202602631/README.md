# Trần Đình Duy — 2A202602631

Phần đóng góp cho K4 Track 4 Capstone: **Cliff Rescue**, mô phỏng tài xế phản ứng muộn, cua/phanh không kịp, xe trượt văng khỏi đường và xoay trên không. Các vòi quanh thân hãm xoay trước, sau đó 4 bộ đẩy dưới gầm nâng xe và hạ cánh an toàn sang bờ sông đối diện.

## Nội dung

```text
TranDinhDuy-2A202602631/
  README.md                   thông tin tác giả và cách chạy
  viewer/
    river.html                giao diện mô phỏng
    river/
      src/                    vật lý, xe, cảnh, cảm biến, điều hướng, hiệu ứng, HUD
      README.md               hướng dẫn sử dụng và module
      INTEGRATION.md          hợp đồng JavaScript/WebSocket cho team
  tools/                      server điều hướng Python mẫu và dependencies
  tests/                      kiểm thử lõi, scenario, adapter và trình duyệt
  evidence/                   ảnh/JSON kiểm thử cục bộ, được gitignore
```

Code mô phỏng, backend mẫu, tài liệu và kiểm thử đều nằm trong thư mục này. Viewer gốc của repo có lối vào cảnh, thư viện Three.js dùng lại từ `viewer/vendor/three.min.js` ở gốc repo.

## Chạy từ thư mục gốc repo

```powershell
python -m http.server 8013 --bind 127.0.0.1
```

Mở **http://127.0.0.1:8013/TranDinhDuy-2A202602631/viewer/river.html**. Serve toàn bộ repo để tải được thư viện Three.js dùng chung.

Nếu muốn thử bộ điều hướng qua WebSocket, mở terminal khác tại gốc repo:

```powershell
python -m pip install -r TranDinhDuy-2A202602631/tools/river-requirements.txt
python TranDinhDuy-2A202602631/tools/river_controller.py
```

Trong giao diện chọn `ws://127.0.0.1:8765` → **Kết nối bộ thật**. Server mẫu chạy mock để xác minh giao thức; teammate có thể thay planner sau.

## Kiểm thử

```powershell
node --test TranDinhDuy-2A202602631/tests/river-core.test.cjs TranDinhDuy-2A202602631/tests/river-adapters.test.cjs TranDinhDuy-2A202602631/tests/rescue-modules.test.cjs TranDinhDuy-2A202602631/tests/skid-scenarios.test.cjs
# Khi HTTP server và WebSocket server đã chạy, cần Playwright + Chromium:
python TranDinhDuy-2A202602631/tests/river_browser_smoke.py
```

3 mẫu: cua muộn/đáp thẳng, đường ướt/tiếp cận vòng, xoay mạnh/giữ cao rồi đáp. Thay thông số sẽ đổi quỹ đạo, vị trí, hướng và thời gian tiếp đất.

[Hướng dẫn chi tiết, phím điều khiển và các phần mock](viewer/river/README.md) · [Tài liệu tích hợp cho team](viewer/river/INTEGRATION.md).
