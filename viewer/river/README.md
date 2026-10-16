# Cliff Rescue — K4 Track 4

Prototype xe cứu hộ trên đường núi, dùng lại đoạn đường và ngã ba của cảnh hiện có. **Tài xế mất tập trung → phanh/cua muộn → lốp trượt → xe văng và xoay → vòi quanh thân hãm xoay → 4 bộ đẩy dưới gầm nâng xe → đáp sang bờ bên kia.** Không có cánh quạt hay tay đòn bên ngoài thân xe.

## Chạy

Từ thư mục gốc dự án:

```powershell
python -m http.server 8013 --bind 127.0.0.1
```

Mở **http://127.0.0.1:8013/viewer/river.html**. Trong viewer gốc cũng có lối vào chương 11. Trang chạy offline, dùng Three.js đã có; cần WebGL và HTTP server vì mã dùng ES modules.

Mặc định tự cứu hộ. Có thể tua demo bằng thanh thời gian. Khi nhận thao tác lái/cứu hộ, demo chuyển sang mô phỏng live tại trạng thái đang xem. Bấm khung cảnh 3D trước khi dùng bàn phím.

| Phím | Thao tác |
|---|---|
| W / S | Ga / phanh |
| A / D | Lái trái / phải |
| Space | Kích hoạt cứu hộ thủ công |
| E | Bật/tắt tự cứu hộ |
| H | Bật/tắt giữ cao độ |
| L | Bắt đầu hạ cánh, chỉ hạ xuống khi ở trên bờ an toàn |
| R | Đặt lại xe |
| P | Chạy/dừng |

Bảng **Thông số & cảm biến thử nghiệm** có trọng lực, công suất đẩy, tia cảm biến, đường rơi/cứu hộ, thử rơi tự do và dịch chuyển đến mép vực. Độ trễ cứu hộ nằm trong bảng **Tai nạn & phương án hạ cánh**.

## Scenario và cách đáp

Chọn **Kịch bản mẫu**, hoặc mở **Tai nạn & phương án hạ cánh** để chỉnh:

| Mẫu | Diễn biến | Cách đáp |
|---|---|---|
| 01 · Cua muộn | Độ bám 45%, phản ứng trễ, xe trượt và xoay | Tiếp cận thẳng khu giữa bờ, hướng 0° |
| 02 · Đường ướt | Độ bám 25%, xung xoay mạnh hơn | Bay vòng, khu đáp lệch z=-6 m, hướng khoảng -17° |
| 03 · Xoay mạnh | Xung xoay 3,1 rad/s, lực nâng mạnh hơn | Giữ cao độ 1,5 s trước khi đáp ở z=+6 m, hướng khoảng +14° |

Các slider điều khiển độ trễ tài xế, độ bám, xung xoay, trễ cứu hộ và vị trí khu đáp. `landingStyle` chọn đáp thẳng / bay vòng / giữ cao rồi đáp. Thông số ảnh hưởng đến mô phỏng thật; không chọn một đoạn phim dựng sẵn. 3 preset được kiểm thử đáp an toàn. Tự chỉnh tốc độ quá cao hoặc cứu hộ quá trễ có thể gây va chạm/rơi xuống nước; phản ứng sớm có thể dừng trước bờ.

Vệt đen và khói lốp hiện lúc trượt. HUD hiển thị công suất vòi quanh thân và tốc độ xoay. File xuất có `landingSummary` với kiểu đáp, thời điểm giữ cao và trạng thái tiếp đất để so sánh.

## Module

```text
river/
  core.js                     public exports, browser + Node
  controllers.js              JavaScript / WebSocket adapters
  src/
    core/                     constants, config, utils, keyboard input
    scene/                    existing road + canyon, lighting, landing zone
    vehicle/                  engine, grip/slip dynamics, 4 lift thrusters, body RCS, state machine
    sensors/                  cliff, altitude, IMU, speed, edge risk, landing, sensor suite
    guidance/                 fall prediction, rescue path, PD stabilization, horizontal guidance
    effects/                  exhaust, paths, warning area
    ui/                       HUD, debug controls
    main.js                   orchestration, playback, integration hooks
```

## Bàn giao cho team

- Cảm biến: `RiverDemo.setSensors({cliff: {sample(context) {...}}})` thay từng module, các cảm biến còn lại giữ nguyên.
- Bộ điều khiển: `RiverDemo.setController({reset(observation), update(observation)})`, hoặc WebSocket. Trả lệnh ga/phanh/lái và 4 lực đẩy; không sửa trực tiếp mesh.
- Xe: `RiverDemo.setVehicleModel(factory)` nhận mesh/GLTF đã tải, giữ `update(state, thrusters, dt, attitudeThrusters)`; tham số thứ tư là lực các vòi quanh thân.
- Cảnh: `RiverDemo.setEnvironmentFactory(factory)` thay assets, `setTerrain(adapter)` thay truy vấn địa hình cho vật lý/cảm biến.
- Hạ cánh: `setLandingZoneProvider(fn)` thay vùng đáp, `setLandingPolicy({evaluate(state, zone)})` thay điều kiện chấp nhận.
- Dự đoán/hướng dẫn: `setPredictor({predict(context)})`, `setGuidance({generate(context)})`.

[Hợp đồng dữ liệu và ví dụ tích hợp](INTEGRATION.md).

**Mock:** mặt đất hai bờ phẳng, địa hình truy vấn giải tích, cảm biến hoàn hảo, mô hình bám/trượt đơn giản, xung va mép gây xoay và gia tốc dịch chuyển ngang khi bay. Vòi quanh thân được quy về mô-men xoay; chưa mô hình hóa đầy đủ phản lực tịnh tiến của mỗi vòi. Đường cyan là hướng dẫn, không ép xe chạy theo spline. Chưa có khí động học, SLAM, nhiễu cảm biến hoặc ROS bridge.

**Đã triển khai để tích hợp:** lực đẩy riêng cho 4 góc với smoothing/clamp, pitch/roll torque + PD, cảm biến và state machine tách module, JavaScript hooks, WebSocket có kiểm tra run/seq và timeout, xuất JSON có sensor/command/thruster ở mỗi bước.

```powershell
node --test tests/river-core.test.cjs tests/river-adapters.test.cjs tests/rescue-modules.test.cjs tests/skid-scenarios.test.cjs
python -m pip install -r tools/river-requirements.txt
python tools/river_controller.py
# Trong giao diện: ws://127.0.0.1:8765 -> Kết nối bộ thật
```

Server Python mẫu vẫn là thuật toán mock, thay `navigate()` bằng bộ của team. Các bản dữ liệu cũ `avsim-viewer/1` của lab phố không áp dụng cho cảnh cứu hộ này.
