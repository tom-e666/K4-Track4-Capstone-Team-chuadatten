MSSV: 202602873
Lab 11 - Scenario Forge
Thị trấn B, theo evidence/meta.json: _run.town.

Câu 1: Phát lại log và thế giới phản ứng

Theo evidence/replay.json, có 400 kịch bản, dropped_human_crash_logs = 0, n_used = 400. Replay ghi nhận 115 va chạm: rear = 104, side = 11, front = 0. Reactive không có va chạm ở cả ba loại; crash_in_both = 0.

Rear chiếm 90,4% số va chạm replay. Planner phanh sớm hơn người lái trong log, nhưng xe sau vẫn đi đúng quỹ đạo đã ghi nên không phanh theo. Đây là va chạm giả do phát lại thiếu phản ứng của tác nhân.

Ví dụ replay_only, scenario = 1: tgT = 0,600 s, replay bị tông đuôi; reactive không va chạm, min_ttc_reactive = 2,407 s. Cùng cảnh nhưng xe sau biết phản ứng thì kết quả khác.

Câu 2: Kịch bản bất khả thi

Trong lab11/feasible.py, is_feasible kiểm tra bốn điều kiện:
Gia tốc ngang (pi/cutDur)^2 x 3,5/2 không vượt mu x 9,81: giới hạn bám khi chuyển làn.
cutBrake không vượt mu x 9,81: giới hạn phanh do ma sát.
pedV không vượt 2,5 m/s: ngưỡng hành vi người đi bộ mà lab chấp nhận.
cutV lớn hơn 0 và không vượt 22 m/s: giới hạn tốc độ xe chuyển làn trong lab.

Theo evidence/hunt.json, số mẫu bị loại ở bus_bypass, cut_in, crosswalk_block, rear_ended lần lượt là 3, 9, 10, 10. Hai cuộc săn cuối đồng hạng cao nhất. CEM chọn mẫu nguy hiểm nên dễ bị kéo về các giá trị cực đoan, kể cả vượt giới hạn vật lý. Nếu vẫn chạy và đếm chúng, xác suất thất bại sẽ bị thổi phồng bởi những cảnh nằm ngoài miền hợp lệ.

Câu 3: Monte Carlo

Theo evidence/mc.json: k = 3, n = 3000, p = 0,001; Wilson 95% = [0,000340; 0,002936]. Có 2 bus_bypass và 1 crosswalk_block.

Để sai số tương đối khoảng ±20% ở mức 95%, số mẫu xấp xỉ 1,96^2/(0,2^2 x p), tức khoảng 96.000 lần chạy với p = 0,001. Đây là tính gần đúng từ p hiện có.

cut_in, rear_ended và other đều chưa xuất hiện. Với mỗi kiểu, mc.json: per_mode ghi k = 0, hi = 0,001279. Chưa thấy trong 3000 lần chạy không có nghĩa xác suất bằng 0.

Câu 4: Importance sampling

Số liệu theo evidence/estimate.json: seeds[].per. Mỗi dòng có n = 600.

Kiểu              Seed   p          hits   ess     top_share
bus_bypass        0      8,46e-4     29     122,5   0,616
bus_bypass        1      6,67e-4     39     121,2   0,171
bus_bypass        2      1,64e-3     37     130,6   0,334
cut_in            0      0           0     132,2   0
cut_in            1      0           0     125,6   0
cut_in            2      0           0     140,1   0
crosswalk_block   0      3,19e-7     73     101,1   0,175
crosswalk_block   1      1,35e-6     87     117,2   0,466
crosswalk_block   2      2,26e-6     85     124,2   0,810
rear_ended        0      0           0     123,1   0
rear_ended        1      0           0     117,0   0
rear_ended        2      0           0     111,0   0

Các dòng p = 0 chỉ là ước lượng điểm khi hits = 0. Cận upper_zero_hits của cut_in và rear_ended là 0,025 cho mỗi seed, tính bằng 3/(0,2 x 600), không dùng 3/ESS.

Total của ba seed là 8,46e-4; 6,68e-4; 1,64e-3, chênh khoảng 2,46 lần. Chúng nằm trong khoảng Wilson của MC, nhưng chưa thể gọi là đồng thuận chặt. Riêng crosswalk_block chênh khoảng 7 lần giữa các seed và thấp hơn cả cận dưới MC của kiểu này là 5,88e-5. Đây là dấu hiệu đề xuất săn bị hẹp hoặc bỏ sót vùng thất bại.

bus_bypass seed 0 và crosswalk_block seed 2 có top_share trên 0,5. Một mẫu đóng góp lần lượt 61,6% và 81,0% ước lượng, nên kết quả rất nhạy với vài trọng số lớn. top_share thấp cũng không bảo đảm đã phủ hết vùng va chạm.

estimate.json: budget = 16800, so với khoảng 96.000 mẫu MC để đạt ±20% ở p = 0,001. Tuy nhiên, các khoảng IS hiện còn rộng và thiếu ổn định; chưa thể kết luận hai ngân sách đạt cùng độ chính xác.

Phần other lấy từ mc.json: k = 0, n = 3000, hi = 0,001279. Nó gom các thất bại chưa có nhãn cơ chế riêng, nên chưa có điểm nguy hiểm riêng để dẫn hướng cuộc săn.

Câu 5: Các kiểu thất bại

bus_bypass: xe C2 chờ khe để lách xe buýt rồi lấn vào làn xe ta. Planner nhận xe lấn làn muộn nên va chạm. Ảnh evidence/shots/shot_bus_bypass.png ghi sự kiện lúc 12,6 s.

crosswalk_block: người đi bộ ra khỏi vùng xe buýt che khuất; xe ta phanh nhưng dừng chắn vạch, buộc người đi bộ đứng lại. Đây là sự kiện block, không phải tiếp xúc thân người. Ảnh evidence/shots/shot_crosswalk_block.png ghi sự kiện lúc 19,5 s.

Theo evidence/hunt.json: found, cuộc săn bus_bypass tìm 110 bus_bypass; cut_in tìm 28 bus_bypass; crosswalk_block tìm 65 crosswalk_block và 1 bus_bypass; rear_ended tìm 4 crosswalk_block và 2 bus_bypass. Không cuộc nào tìm được cut_in hay rear_ended đúng nhãn.

Tôi chỉ kết luận chưa tìm thấy trong ngân sách này. MC cho cận trên 0,001279 với hai kiểu đó; IS cho cận 0,025 mỗi seed. Một cuộc săn chung dễ tập trung vào kiểu dễ tìm rồi bỏ sót kiểu khác; ngay cuộc săn cut_in cũng đã bị hút sang bus_bypass.

Câu 6: Sửa planner

Theo evidence/fix.json: chosen.policy, policy được chọn là cutinLat = 3,0, tăng từ 2,15 để nhận xe lấn làn sớm hơn. Chọn trên TRAIN, sau đó đánh giá riêng trên TEST.

Tập và policy     fail       fail_hi    progress (m)   harsh
TRAIN gốc        8,17e-4    1,69e-3     211,79          8,20
TRAIN chọn       7,03e-7    1,19e-6     211,78          7,80
TEST gốc         4,91e-4    2,10e-3     214,35          8,56
TEST chọn        6,47e-6    3,88e-2     214,25          8,08

Nguồn: baseline_train, chosen, baseline_test, chosen_test. harsh là số khung hình phanh dưới -3 m/s² trung bình mỗi kịch bản, không phải số đợt phanh riêng biệt.

Trên TEST, fail_hi_seen của bản gốc là 8,20e-4, bản chọn là 1,41e-5. Bản chọn không thấy bus_bypass nên phải cộng cận 3/(0,2 x 400) = 0,0375, cùng cận other = 0,001279, vào fail_hi. Vì vậy cận đầy đủ có thể lớn hơn fail của gốc dù ước lượng điểm giảm. Dữ liệu cho thấy cải thiện, nhưng chưa đủ chứng nhận rủi ro nhỏ.

Ứng viên cautionV = 6,0 nghe thận trọng hơn nhưng TRAIN fail = 1,07e-3, cao hơn gốc 8,17e-4; bus_bypass top_share = 0,793. Giảm tốc làm thay đổi khe và thời điểm C2 quyết định nhập làn, nên tác nhân khác có thể tạo ra tình huống mới. Kết quả này còn chịu ảnh hưởng lớn của trọng số; cần xem lại quỹ đạo để xác nhận chuỗi sự kiện cụ thể.

Ảnh evidence/shots/shot_ghost_fix.png chụp tại kịch bản B-01 (t = 12,6 s) đã bật bóng mờ: xe chạy chính sách gốc lao lên va chạm với xe C2, trong khi chiếc bóng mờ bán trong suốt (bản sửa cutinLat = 3,0) đã phanh dừng an toàn phía sau, minh hoạ trực quan bản sửa cứu được tình huống lách xe buýt này.

Câu 7: Từ mô phỏng ra đường thật

Khe chuyển làn: avsim/space.py đặt trung vị cutGap của Town B là 12 m. Tôi sẽ thay bằng quỹ đạo xe từ camera trên cao tại đoạn vượt xe buýt, đo khe thực tế lúc bắt đầu chuyển làn. Nếu thực tế chấp nhận khe hẹp hơn, mô phỏng có thể đánh giá thấp rủi ro do xe lấn làn.

Quan sát của người đi bộ: pedLook có xác suất 0,85. Cần video tại trạm buýt, ghi người có ngó qua vật che trước khi bước xuống đường hay không. Nếu tỷ lệ quan sát thực tế thấp hơn, mô phỏng có thể đánh giá thấp các tình huống bất ngờ sau xe buýt.

Mặt đường: Town B đặt xác suất đường ướt là 0,30; mu = 0,5 khi ướt và 0,8 khi khô. Cần đo ma sát trên tuyến thử, ghép với thời tiết và số chuyến thực tế. Nếu đường trơn hơn hoặc tỷ lệ chuyến đi trên đường ướt cao hơn giả định, quãng phanh tăng và rủi ro có thể bị đánh giá thấp.

Câu 8: Giới hạn

Xác suất này chỉ áp dụng cho hình học một con phố trong lab, chưa đại diện cho các tuyến đường và nút giao khác.
Hành vi xe chủ yếu theo IDM và luật chấp nhận khe, chưa phản ánh đủ khác biệt giữa người lái thực tế.
Nhận thức gần như hoàn hảo khi không bị che khuất; chưa có nhiễu đo, bỏ sót và độ trễ phát hiện như cảm biến thật.

Nếu có thêm một tuần, tôi sẽ đo mức thay đổi rủi ro khi thêm nhiễu và độ trễ nhận thức, chạy nhiều seed hơn cho crosswalk_block, đồng thời bổ sung căn cứ thực tế cho exposure của tình huống mới.

Câu 9: Rắc rối tự dựng

Tôi chọn người đi bộ H0 băng ngang ngoài vạch tại x = 95 m, từ lề nam. Đây là tình huống người qua đường trước khu vực trạm buýt. Planner có logic thận trọng quanh vạch chính, nhưng vẫn có thể phản ứng muộn với người băng ngang ở vị trí khác.

Theo hazard.json: space, h0_x trong [92; 98] m, h0_v trong [1,3; 1,9] m/s, h0_trig trong [38; 52] m; exposure = 0,035. Trường why mới mô tả dải giả định và tần suất ước tính, chưa dẫn dữ liệu đo thực tế. Vì vậy kết quả mỗi chuyến dưới đây phụ thuộc vào giả định exposure này.

Theo evidence/mine.json: world = town, conditional_on là giao thông thị trấn của tôi. Có 1488/1500 mẫu hợp lệ, accepted = 0,992; infeasible = 12. Cảnh dựng ban đầu có tiếp xúc ego-H0 lúc 12,35 s, theo exact.planner. Ảnh: evidence/shots/shot_hazard_h0.png.

Policy     crashes   avoidable   unavoidable   unattributed   low_speed
Gốc        58        56          1             1              4
Chọn       56        56          0             0              4

Nguồn: mine.json: baseline và chosen. avoidable là tránh được khi phanh tối đa ngay từ lúc nhìn thấy rắc rối đã bắt đầu; unavoidable vẫn va chạm khi phanh như vậy; unattributed xảy ra trước khi rắc rối bắt đầu. low_speed là số vụ xe ta dưới 1 m/s, đã nằm trong các nhóm trên, không cộng thêm.

Cả hai policy có p = 56/1488 = 0,03763 cho lỗi planner khi gặp rắc rối; Wilson 95% = [0,02909; 0,04856]. Nhân exposure cho per_trip = 0,001317, per_trip_hi = 0,001699. Mức này cùng bậc với ước lượng bus_bypass ở estimate.json là 6,67e-4 đến 1,64e-3, nhưng không thể xếp hạng chắc vì IS còn dao động và exposure chưa được kiểm chứng.

cutinLat = 3,0 giảm tổng va chạm từ 58 xuống 56 nhưng số avoidable vẫn là 56. Bản sửa ở câu 6 chưa giải quyết được lỗi chính của rắc rối này.

Chưa có log tự lái trong bằng chứng để kết luận tôi tránh được. Nếu thử, tôi sẽ giảm ga và chuẩn bị phanh khi thấy người có hướng bước vào đường; cần ghi lại lượt lái để so sánh.

Chuẩn avoidable khá lạc quan vì oracle phanh ngay, không có trễ nhận thức hay quyết định. Ngược lại, nó chỉ thử phanh nên có thể bỏ qua các trường hợp tránh được bằng đánh lái. Tôi đề xuất thêm dự đoán đường đi và ý định người đi bộ ngoài vạch, rồi điều chỉnh tốc độ trước khi họ vào hành lang xe. Chưa có kết quả để khẳng định mọi cách chỉnh chín núm hiện tại đều không hiệu quả.
