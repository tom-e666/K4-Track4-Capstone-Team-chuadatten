# Hợp đồng tích hợp Cliff Rescue

[Chạy, bàn phím và cấu trúc thư mục](README.md). Mã dùng ES modules, không cần bundler. `river.html` giữ đoạn đường/nhánh ngã ba hiện có và dựng thêm vực sâu, núi và khu đáp.

## Tọa độ và bước mô phỏng

Đơn vị m, s, m/s, m/s², radian, N. +x là hướng xe ban đầu / đi qua vực; +y lên; +z phía phải/nam. `y=0` là bánh xe trên mặt đất. Pitch dương ngẩng mũi; roll dương nâng bên trái. Vật lý chạy `dt=0.02` (50 Hz), độc lập tốc độ render.

4 emitter `FrontLeftThruster`, `FrontRightThruster`, `RearLeftThruster`, `RearRightThruster` gắn dưới gầm tại x=±1.45 m, z=±0.72 m. Không có bộ phận quay. Công suất từng vòi chuẩn hóa 0..1; lực mỗi vòi = power × maxLift × mass / 4. `maxLift` là gia tốc nâng tổng tối đa. Hover bằng gravity/maxLift khi thân nằm ngang.

Thêm **8 vòi quanh thân**: 4 vòi ở các góc tạo hai cặp yaw đối nhau, 2 vòi pitch trước/sau và 2 vòi roll trái/phải. `attitudeThrusters.js` nhận 6 kênh `yawPositive/yawNegative/pitchPositive/pitchNegative/rollPositive/rollNegative`, mỗi kênh 0..1, smoothing 45 ms. Hai phía đối nhau tạo mô-men có dấu; yaw được hãm tốc trước rồi mới chỉnh hướng để tránh đổi chiều điều khiển đột ngột ở ±π. Các vòi này xuất hiện trong model dưới tên `*Jet` và được hiển thị độc lập với 4 vòi nâng `*Thruster`.

Công suất thực được làm mượt với hằng số thời gian 70 ms. Tổng lực có hệ số cos(pitch) × cos(roll). Chênh lệch cặp trước/sau tạo gia tốc pitch, chênh lệch trái/phải tạo gia tốc roll. Bộ PD tạo phân phối lực cân bằng. Lực tiến/phanh ngang khi bay là actuator mock; thay `vehicle/engine.js` khi nối rigid body đầy đủ.

`drivingDynamics.js` giới hạn tốc độ đổi hướng của vận tốc bằng grip × gravity / speed. Hướng thân xe vẫn đổi theo đánh lái nên có thể lệch khỏi hướng trượt. Khi văng khỏi mép, `spinIntensity` tạo xung vận tốc góc trên 3 trục; sau đó tư thế được tích phân từ mô-men/vận tốc góc, không xoay mesh bằng hoạt cảnh. Auto rescue mặc định **đợi xe văng**, rồi chờ `rescueDelay` trước khi bật; không bay lên khi xe vẫn chưa trượt. Manual rescue có thể can thiệp sớm.

## Observation

API `RiverDemo.getObservation()` và request WebSocket có cấu trúc:

```js
{
  schema: 'river-nav/1', type: 'observation', runId: 'rescue-...', seq: 230, dt: .02,
  state: {
    t, x, y, z, vx, vy, vz, speed, pitch, roll, yaw, pitchRate, rollRate, yawRate,
    grounded, flightActive, rescueTriggered, autoRescueEnabled, hoverMode, forceLanding,
    mode, phase, terminal, outcome
  },
  sensors: {
    time, forwardSpeed, altitudeAboveGround, verticalVelocity, pitch, roll, yaw,
    angularVelocity: {pitch, roll, yaw},
    distanceToGroundAhead, roadAheadExists, cliffRiskScore,
    groundDetectedBelow, surfaceDetectedBelow, surfaceBelow,
    edgeDistance, edgeRiskScore, grounded,
    wheelContactState: {frontLeft, frontRight, rearLeft, rearRight},
    landingZoneFound, landingZonePosition: {x,y,z} | null,
    landingZoneNormal: {x,y,z} | null, landingZoneDistance,
    forwardProbes: [{from:{x,y,z}, to:{x,y,z}, hit, safe, distance}],
    downwardProbes: [{from:{x,y,z}, to:{x,y,z}, hit}]
  },
  prediction: {predictedFallPath:[{x,y,z,t}], predictedImpactPoint, timeToImpact},
  guidance: {rescuePath:[{x,y,z}], targetHoverPoint, targetLandingPoint},
  thrusters: {frontLeft, frontRight, rearLeft, rearRight, totalLift, totalForceN,
             pitchAcceleration, rollAcceleration},
  map: {nearBankX, farBankX, waterY, junctionX, roadHalfWidth,
        landing: {x,y,z,halfX,halfZ} | null},
  vehicle: {length, width, mass, leverX, leverZ, pitchInertiaPerMass,
            rollInertiaPerMass, maxBrake, maxLift, gravity, flightAvailable},
  scenario: {brakeDistance, initialSpeed, rescueThreshold, hoverHeight},
  metrics: {distanceToEdge, stoppingDistance}
}
```

Khoảng cách mép lấy từ đầu xe; bắt đầu mất hỗ trợ ở `x>-length/2`. Chạm bờ đối diện đòi hỏi toàn xe qua bờ. `altitudeAboveGround` đo đến bề mặt truy vấn, gồm cả mặt nước; `groundDetectedBelow` chỉ true với đất, `surfaceBelow` phân biệt đất/nước. Truy vấn không có hit trả altitude null. Forward probes phát hiện đứt bề mặt trong khoảng 12–45 m, `roadAheadExists` ở đây nghĩa là có hỗ trợ mặt đất an toàn phía trước, không phải bộ phân loại làn đường.

Đường cam là dự đoán đạn đạo không có lực cứu hộ/phanh, bắt đầu từ mép khi xe còn tiếp đất. Impact được tính ở mặt nước hoặc vách vực đối diện. Đường cyan là đường hướng dẫn đến mục tiêu và vệt xe đã đi, không dùng để đặt trực tiếp vị trí xe.

Các trường bổ sung cho kịch bản trượt:

- `state`: `driverAttentionSince`, `driverReacted`, `skidding`, `slipAngle`, `accidentOccurred`, `airborneSince`, `rescueSince`, `hoverSince` (mốc chưa xảy ra là null).
- `scenario`: thêm `driverDelay` (s), `grip` (0.15..1), `spinIntensity` (0..4 rad/s), `rescueDelay` (0..1.4 s), `landingStyle` (`direct|arc|hover`), `landingOffset` (-12..12 m).
- `attitudeThrusters`: 6 mức lực thực, `yawAcceleration/pitchAcceleration/rollAcceleration` (rad/s²), `totalPower` (trung bình 6 kênh).
- `guidance`: thêm `targetWaypoint`, `targetAltitude`, `targetYaw`, `holding`, `landingStyle`.
- `map.landing.heading`: hướng đáp mục tiêu (rad).

`driverDelay` tính từ lúc xe đi qua x=-35 m đến phản ứng đánh lái. Controller mẫu phanh muộn và cố rẽ trái tại ngã ba. `rescueDelay` tính từ mất hỗ trợ bờ; nhịp bật vòi quanh thân đến trước giai đoạn tăng lực nâng chính khoảng 180 ms.

## Commands

```js
{
  throttle: 0, brake: .5, steer: 0, flight: true, lift: .5,
  thrusters: {frontLeft:.55, frontRight:.55, rearLeft:.45, rearRight:.45}
}
```

`throttle`, `brake`, `lift` trong 0..1; `steer` -1..1, dương hướng +z. `flight` boolean. Có `thrusters` thì phải đủ 4 số, chúng ghi đè `lift`; không có thì engine tự mix lift bằng PD để cân bằng thân xe. Giá trị hữu hạn vượt miền được clamp; NaN/Infinity/chuỗi/thiếu vòi bị từ chối. Số thiếu ở lệnh cấp cao mặc định 0, flight mặc định false. Lệnh flight vẫn bị vô hiệu nếu hệ đẩy bị tắt trong kịch bản.

Lệnh nâng cao cho xe đang xoay:

```js
{
  flight:true,
  acceleration:{x:-5,z:2}, // gia tốc ngang trong hệ thế giới, m/s²
  attitude:{yawPositive:.8,yawNegative:0,pitchPositive:0,pitchNegative:.2,rollPositive:.1,rollNegative:0},
  thrusters:{frontLeft:.5,frontRight:.5,rearLeft:.5,rearRight:.5}
}
```

`acceleration.x` clamp -10..8, z clamp -8..8. Khi có acceleration và flight đang bật, nó thay actuator ngang ga/phanh, tách hướng chuyển động khỏi yaw thân xe đang xoay. Khi không có, lệnh ga/phanh cũ vẫn được hỗ trợ. `attitude` có kênh thiếu mặc định 0; nếu thiếu toàn bộ thì engine dùng PD mặc định để cân bằng về yaw=0. Controller JS và Python mẫu đều dùng các trường mới; teammate có thể điều khiển đồng thời lực nâng, mô-men quanh thân và hướng dịch chuyển.

## JavaScript controller

```js
RiverDemo.setController({
  name:'Team navigator',
  reset(initialObservation) { /* reset memory */ },
  update(observation) {
    return RiverCore.mockCommand(observation); // thay thuật toán thật ở đây
  },
  dispose() { /* tùy chọn */ }
});
```

`update` đồng bộ, có thể được gọi nhiều lần trên cùng seq, vì vậy dùng `seq/state.t`, không dùng số lần gọi để đếm thời gian. Trả null để chờ; Promise không hỗ trợ. Không tự fallback về mock khi lỗi. `RiverDemo.useMock()` trở lại demo. Các ý định Space/H/L/E được truyền trong state; controller ngoài quyết định cách phản hồi chúng.

## WebSocket

```powershell
python -m pip install -r TranDinhDuy-2A202602631/tools/river-requirements.txt
python TranDinhDuy-2A202602631/tools/river_controller.py
```

Mở giao diện, bấm **Kết nối bộ thật** với `ws://127.0.0.1:8765`. Đây là server kiểm tra giao thức chạy mock; thay `navigate(observation)` bằng planner của team, giữ envelope:

```json
{"schema":"river-nav/1","type":"command","runId":"rescue-...","seq":230,
 "controls":{"flight":true,"thrusters":{"frontLeft":0.5,"frontRight":0.5,"rearLeft":0.5,"rearRight":0.5}}}
```

Request tối đa 20 Hz, một request đang chờ. Phải echo chính xác runId/seq. Response sai phiên/seq/schema bị bỏ qua. Giữ lệnh gần nhất tối đa 500 ms thực; mất kết nối hoặc lệnh quá hạn làm dừng thời gian mô phỏng. Request chờ được thử lại sau 1 s. Đây là cơ chế chờ của demo, không phải failsafe cho xe thật. ROS/C++ cần server/cầu nối cùng giao thức, chưa có ROS bridge sẵn.

## Thay cảm biến / địa hình / vùng đáp / model

Mỗi cảm biến triển khai `sample(context)`, trả các trường do nó sở hữu. Context gồm snapshot state, config, terrain và landingZone. Ví dụ thay sensor IMU:

```js
RiverDemo.setSensors({
  imu: { sample({state}) {
    return {pitch:state.pitch, roll:state.roll, yaw:state.yaw,
      verticalVelocity:state.vy, angularVelocity:{pitch:state.pitchRate,roll:state.rollRate,yaw:state.yawRate}};
  }}
});
```

Các slot: `cliff`, `altitude`, `imu`, `speed`, `edge`, `landing`. Cảm biến không được thay dùng mặc định. Khi đọc dữ liệu bất đồng bộ, giữ frame cache trong sensor và trả bản mới nhất ở sample().

Các hook sau reset lần chạy, trừ model/cảnh chỉ thay hiển thị:

| Hook | Hợp đồng |
|---|---|
| `setTerrain(adapter)` | `surfaceAt(x,z,config)` -> `{y,normal,kind:'ground'|'water',road}` hoặc null; `supportAt(state,config)` -> boolean; `edgeDistance(state,config)` -> number |
| `setLandingZoneProvider(fn)` | `fn(config)` -> `{id,enabled,position,normal,halfX,halfZ}` hoặc null |
| `setLandingPolicy(policy)` | `evaluate(state,zone)` -> boolean, chấp nhận sau khi xe đã tiếp đất và dừng |
| `setPredictor(module)` | `predict(context)` -> prediction theo schema trên |
| `setGuidance(module)` | `generate(context)` -> guidance theo schema trên |
| `setVehicleModel(factory)` | `factory(THREE)` -> `{group,update(state,thrusters,dt,attitudeThrusters),dispose?}`; có thể trả GLTF đã tải, tham số thứ tư tùy chọn cho model cũ |
| `setEnvironmentFactory(factory)` | `factory(THREE,scene)` -> `{world,build(config),update(time),dispose?}` |

Terrain vật lý và scene hiển thị là hai adapter riêng; thay mesh chưa tự đổi queries. Engine hiện hỗ trợ mặt đất hai bờ y=0; vùng đáp ở cao độ/nghiêng khác cần mở rộng physics/contact trong engine. Raycasts trong Three.js có thể được cài bên trong terrain adapter để thay truy vấn giải tích hiện tại.

Dùng Node không cần DOM: `const {Engine,mockCommand}=await import('./TranDinhDuy-2A202602631/viewer/river/core.js')`. Constructor thứ hai của Engine nhận dependencies `{terrain,sensors,predictor,guidance,landingPolicy,landingZoneProvider}`, nên có thể tích hợp và test ngoài giao diện.

## State machine và xuất dữ liệu

DRIVING/WARNING → LATE_TURN → SKIDDING → SPINNING → RESCUE_ACTIVE → RCS_STABILIZE/HOVER_STABILIZE → MOVE_TO_SAFE_ZONE → LANDING → LANDED. Với kiểu hover, thêm HOLD_BEFORE_LANDING trong 1,5 s trước khi hạ xuống. FALLING dùng cho mất hỗ trợ không có xung tai nạn. Lỗi kết thúc ở FAILURE; dừng trước vực ở IDLE. H giữ hover, L yêu cầu đáp và chỉ cho hạ về đất khi qua bờ an toàn. Auto rescue sử dụng `airborneSince` và `rescueDelay`; `rescueThreshold` giữ trong schema để các module của team dùng, controller tai nạn mặc định không dùng nó để bay sớm.

Kết quả: landed / splash / crashed / stopped / off_target / out_of_bounds. Va chạm nếu tốc độ tiếp đất đứng >4.5 m/s, ngang >13 m/s, pitch/roll >0.6 rad, hoặc lao vào vách dưới mặt đất. Đáp thành công yêu cầu dừng dưới 0.4 m/s, trong vùng đáp và policy chấp nhận. Giới hạn 40 giây mô phỏng.

Xuất JSON **river-run/2**: config, controller, mode, dt, completed/outcome, events và frames đến thời điểm đang xem. Mỗi frame có toàn observation, sensor, prediction, guidance, power và command. Giao thức truyền **river-nav/1** giữ tương thích các trường cấp cao trước đây và bổ sung sensors/guidance/thrusters. Consumer run/1 cũ cần cập nhật cho run/2.

`landingSummary` thêm `style`, `touchdown` và `holdStartedAt`. Sự kiện touchdown lưu x/y/z, yaw/pitch/roll, vận tốc ngang/đứng và thời gian. Ba preset trong `core/scenarios.js` đều kiểm thử hạ cánh nhưng có vị trí, hướng và thời gian khác nhau. Tự chỉnh các trường hợp cực đoan có thể thất bại, không ép kết quả hạ cánh.

## Kiểm thử

```powershell
node --test TranDinhDuy-2A202602631/tests/river-core.test.cjs TranDinhDuy-2A202602631/tests/river-adapters.test.cjs TranDinhDuy-2A202602631/tests/rescue-modules.test.cjs TranDinhDuy-2A202602631/tests/skid-scenarios.test.cjs
# HTTP + WebSocket server mẫu đang chạy:
python -m pip install playwright
python -m playwright install chromium
python TranDinhDuy-2A202602631/tests/river_browser_smoke.py
# Hoặc thêm --browser "đường-dẫn/chrome.exe" nếu đã cài Chromium
```

Ảnh và lần chạy test ở TranDinhDuy-2A202602631/evidence/river/ (gitignore). Bao gồm mô phỏng ngang vực, lực/mô-men từng vòi, hồi phục pitch/roll, hover, cảm biến thay thế, bàn phím, mobile/laptop, export và WebSocket thật. Thư viện Three.js được lưu sẵn trong viewer/vendor; không phụ thuộc CDN hoặc assets thương hiệu.
