# Lab 11 · Scenario Forge — lò rèn kịch bản

**Track 4 · CV & Robotics · Day 11: Synthetic, simulated & trajectory data.** Cá nhân, 4 giờ, laptop hoặc Colab, chỉ cần `numpy` và `pytest`. Không GPU, không API key.

Bạn là kỹ sư kiểm thử của một đội xe tự lái. Planner của đội chạy trong một thị trấn mô phỏng: một con phố, một xe buýt đỗ trước vạch qua đường, một xe hàng xóm muốn chuyển làn, một người đi bộ, một xe bám đuôi. Trên 1 000 lần chạy ngẫu nhiên, planner gần như không bao giờ va chạm. Câu hỏi của bạn:

1. **Nó va chạm bao lâu một lần, thật sự?** Với xác suất cỡ 10⁻³ hay 10⁻⁵, đếm thô (Monte Carlo) cần hàng chục nghìn lần chạy để thấy vài vụ.
2. **Nó va chạm theo những kiểu nào?** Một thuật toán tìm kiếm dễ tìm thấy một kiểu rồi lặp mãi kiểu đó.
3. **Sửa planner thì có thật sự đỡ hơn không**, hay chỉ đỡ trên đúng những kịch bản bạn đã nhìn?
4. **Rắc rối nào chưa ai nghĩ tới?** Bạn tự dựng một tình huống mới, rồi đo xem nó nguy hiểm đến đâu.

![Trình xem 3D, chương 9: một va chạm "lách qua xe buýt" do cuộc săn CEM tìm ra, chạy lại đúng lúc va chạm](images/ch9-lab-bus-bypass.jpg)

Mọi thứ bạn tìm thấy bằng Python đều xem lại được trong **trình xem 3D**: chương 9 "phòng lab" chạy lại từng kịch bản bằng chính lõi mô phỏng này, khớp từng bit với Python. Chương 10 "xưởng kịch bản" cho bạn tự thêm rắc rối, chỉnh thế giới và planner, hoặc tự cầm lái.

## Thế giới

`avsim` là bản numpy của lõi mô phỏng trong trình xem 3D (`sim-core.js`). Một lần gọi chạy cả **batch** K kịch bản cùng lúc, khoảng 4 ms/kịch bản. `tests/test_parity.py` kiểm tra Python và JS cho cùng kết quả (cần `node`; không có thì test tự bỏ qua).

- Các xe khác lái theo **IDM** và cư xử như người: xe chuyển làn chờ một khe đủ rộng (*gap acceptance*), dừng nhường người qua đường, rẽ phải thì nhường; người đi bộ chờ đến khi xe gần nhất mà họ **nhìn thấy** còn đủ xa. Chỉ xe buýt đủ cao để che tầm nhìn.
- Người đi bộ **không bao giờ bước vào thân xe**: thân xe chắn ngay trước mặt thì họ đứng lại. Nếu xe ta dừng mà thân xe nằm trên vạch qua đường, chắn lối người đang qua, đó là một thất bại tính như va chạm: sự kiện `kind == "block"`, kiểu `crosswalk_block` (planner phanh quá muộn). Va chạm thật có `kind == "contact"`.
- **Xe ta (ego)** chạy một planner có 9 núm chỉnh (`avsim.POLICY`): IDM (`v0, T, a, b, s0`), tốc độ thận trọng gần vạch bị che (`cautionV`, `cautionRange`), biên an toàn khi dự đoán người đi bộ (`pedBuffer`), và ngưỡng ngang để coi một xe đang lấn sang là xe phía trước (`cutinLat`).
- Mỗi kịch bản là một **vector ẩn u**: 11 toạ độ liên tục chuẩn tắc N(0, 1) và 3 toạ độ bật/tắt (đường ướt, người đi bộ có ngó qua xe buýt không, xe chuyển làn có phanh ngay sau khi vào làn không). `space.to_params` đổi u thành tham số mô phỏng. **Phân phối tự nhiên** p(u) của thị trấn bạn suy ra từ MSSV (một trong 4 thị trấn A–D).

Các phân phối này là **giả định dạy học**, không khớp với dữ liệu thật. Câu 7 của báo cáo hỏi cần dữ liệu thật nào để thay từng cái.

## Cài đặt

```bash
pip install -r requirements.txt            # numpy, pytest
# dòng đầu REPORT.md: MSSV: 2xxxxxxx
python -m pytest -q tests/test_todos.py    # 31 test, đỏ hết cho đến khi bạn điền TODO
python run_all.py world                    # kiểm tra lõi mô phỏng (không cần TODO nào)
```

**Trình xem 3D** đi kèm lab: mở `viewer/index.html` bằng trình duyệt (chạy offline, không cần mạng). `tests/test_parity.py` mất 20–90 s và không in gì trong lúc chạy: chờ, nó không treo.

## Các bước

| Bước | Lệnh | Bạn viết (10 TODO) | Bằng chứng |
|---|---|---|---|
| 0 | `run_all.py world` | — | `world.json`: câu chuyện mặc định của trình xem |
| A | `run_all.py replay` | `metrics.min_ttc`, `metrics.contact_type` | `replay.json`: phát lại log vs tác nhân phản ứng trên ~400 kịch bản |
| B | (dùng ở mọi bước sau) | `feasible.is_feasible` | số kịch bản bất khả thi trong `hunt.json` |
| C1 | `run_all.py mc` | `estimate.wilson_ci` | `mc.json`: 3 000 lần chạy tự nhiên |
| C2 | `run_all.py hunt` | `search.cem_update`, `modes.label_mode` | `hunt.json`, `cloud.json`: một cuộc săn CEM cho mỗi kiểu thất bại |
| C3 | `run_all.py estimate` | `estimate.log_weights`, `estimate.is_estimate` | `estimate.json`: importance sampling theo từng kiểu, 3 seed |
| E | `run_all.py fix` | `metrics.progress_metrics`, `policy.choose_policy` | `fix.json`: chỉnh trên TRAIN, báo cáo trên TEST |
| G | `run_all.py mine hazard.json` | — (bạn thiết kế; cần TODO 3, 4 đã xong) | `mine.json`: rắc rối của bạn, quét trên cả dải, so policy gốc với bản sửa |
| F | `run_all.py export` | — | `viewer.json`: mở trong trình xem 3D, chương 9 |

Mỗi bước đọc bằng chứng của bước trước: `estimate` cần `hunt` (và `mc` cho phần va chạm không thuộc kiểu nào), `fix` và `export` cần `hunt`, `mine` dùng policy đã chọn ở `fix` nếu có. Thiếu file thì lệnh dừng và nói cần chạy bước nào trước. Một bước cần TODO chưa làm thì dừng với một dòng nói TODO nào; riêng `replay` vẫn chạy và để trống các trường cần TODO 1, 2, 9 (kèm cảnh báo; con số khi đó chưa có nghĩa).

`python run_all.py all` chạy hết (gồm G nếu có `hazard.json`), khoảng 9 phút trên laptop M-series rảnh, 25 phút nếu máy đang bận việc khác (bước `fix` dài nhất, 3–10 phút), lâu hơn trên Colab. Chạy từng bước một nếu terminal của bạn có giới hạn thời gian. Thêm `--quick` khi đang thử (ngân sách nhỏ, **không** dùng để nộp).

Gợi ý thời gian: A + B 40 phút · C1 + C2 60 phút · C3 + xem 3D 40 phút · E 35 phút · G 40 phút · báo cáo viết dần trong lúc chờ máy chạy.

### A · Phát lại hay phản ứng

Cùng một kịch bản, ba cách: người lái thật (log), planner trên **log phát lại** (xe khác chạy đúng như đã ghi), planner trong **thế giới phản ứng**. Log nào có chính người lái va chạm thì bỏ, vì log thật không chứa tai nạn của chính nó. Đếm va chạm và loại tiếp xúc (`front` / `rear` / `side`).

### C · Đo một xác suất hiếm cho trung thực

- **C1 Monte Carlo.** Đếm k va chạm trong n lần, báo khoảng **Wilson**, không phải khoảng chuẩn: với k = 0 khoảng chuẩn là [0, 0], tức "không bao giờ", và điều đó sai.
- **C2 Săn bằng cross-entropy (CEM).** Mỗi vòng: lấy mẫu từ đề xuất q, chạy, giữ 10% nguy hiểm nhất theo **điểm nguy hiểm của kiểu đó** (TTC tới xe chuyển làn, khoảng phanh còn lại trước người đi bộ, TTC của xe bám đuôi), khớp lại q. Kịch bản bất khả thi không bao giờ được chạy hay đếm.
- **C3 Importance sampling.** Rút mẫu từ **hỗn hợp phòng thủ** q = 0,8·CEM + 0,2·tự nhiên, gán trọng số w = p/q, ước lượng P(kiểu m) = mean(w·y). Các kiểu chia nhau tập va chạm; va chạm không thuộc kiểu nào (`other`) không được săn, nên phần của nó lấy từ Monte Carlo C1 (`estimate.json: other`). Tổng = các kiểu + `other`.
- **Kiểu không có va chạm nào** thì báo cận trên, không báo 0: `upper_zero_hits` = 3/(0,2·n). Vì mọi mẫu rút từ q ≥ 0,2·p, cận này đúng ở 95 %. **Không** dùng 3/ESS: ESS đo trọng số lệch đến đâu, không phải số lần thử độc lập.
- Khoảng ±1,96·se của IS chỉ là **xấp xỉ**: với vài mẫu nặng ký, nó có thể hẹp sai. `top_share` thấp **không** chứng minh ước lượng đúng (q có thể bỏ sót hẳn một vùng va chạm, khi đó không mẫu nào nặng ký cả).

Ba cái bẫy bạn sẽ gặp và phải báo cáo:

1. CEM **co về một kiểu**: săn mãi một chỗ. Vì thế mỗi kiểu có một cuộc săn riêng.
2. Săn càng lâu, q càng hẹp. Ước lượng vẫn không chệch về lý thuyết, nhưng thường bị **thấp**: một vài mẫu nặng ký quyết định cả con số. `top_share` (phần ước lượng do một mẫu nặng nhất mang) cho bạn biết khi nào điều đó xảy ra. Vì vậy ước lượng dùng **đề xuất đầu tiên** mà elite đã toàn là va chạm. Kể cả vậy, ở ngân sách này con số thường thấp hơn sự thật khoảng 1,5–4 lần, và các seed có thể chênh nhau vài lần.
3. Cuộc săn tìm ra **một** cách va chạm, không nhất thiết là cách **hay xảy ra nhất**. Điểm nguy hiểm kéo q về chỗ va chạm dễ nhất, và chỗ đó có thể rất hiếm ngoài đời. Nếu hai seed cho `crosswalk_block` chênh nhau hàng chục lần, hoặc Monte Carlo thấy một vụ mà IS bảo cỡ 10⁻⁶, đó là dấu hiệu.

### E · Sửa planner, kiểm tra trung thực

`policy_candidates.json` liệt kê các bộ núm chỉnh (bạn được thêm; giá trị ngoài `POLICY_LIMITS` trong `run_all.py` bị loại, vì đó không còn là một planner thật). `fix` chấm từng bộ trên cùng một mẫu **TRAIN**, chọn bằng `choose_policy` (cận trên nhỏ nhất, mất tối đa 5% quãng đường), rồi đo bộ được chọn trên **TEST**: seed mới, và một cuộc săn mới nhắm vào chính policy đó.

Mẫu TRAIN được rút ở nơi **policy gốc** thất bại. Một policy mới có thể thất bại ở chỗ khác, nên "0 va chạm trên TRAIN" không chứng minh gì: cột `unseen` đánh dấu những kiểu đó. TRAIN chỉ dùng để sàng; con số bạn báo là con số TEST. Trên TEST:

- kiểu `unseen` cộng cận `upper_zero_hits` của nó vào `fail_hi`;
- **mọi** va chạm khác của xe ta (cắt làn, bị tông đuôi, `other`), đếm trên một tập tự nhiên riêng, cộng vào `fail` và `fail_hi`: một policy chỉ dời thất bại sang kiểu không ai săn sẽ không trông an toàn;
- quãng đường tính **đến lúc va chạm**, không tính phần xe "chạy tiếp" sau đó.

Vì thế TEST có hai con số. `fail_hi_seen` chỉ cộng các kiểu có va chạm: đó là cái dữ liệu cho thấy. `fail_hi` là cận trên 95 % **hợp lệ** cho cả policy: nếu bản sửa xoá hẳn một kiểu, cận của kiểu đó vẫn là 3/(0,2·n), khoảng 15/n. Với n = 400, không ngân sách nào ở lab này chứng nhận được một kiểu đã biến mất dưới 10⁻². Đó là bài học, không phải lỗi: muốn chứng nhận p < 10⁻³ thì cần cỡ 15 000 lần chạy trên đúng chỗ đó.

Grader chạy lại TEST với một seed bí mật, nên chọn ứng viên theo số TEST công khai không có lợi.

Một cái bẫy của TRAIN: ứng viên mà mọi kiểu đều `unseen` có `fail_hi` = 0 trên TRAIN, nên `choose_policy` thích nó, kể cả khi nó chạy nhanh hơn và nguy hiểm hơn. TEST sẽ lộ ra (cận của kiểu `unseen` là 3/(0,2·n)). Vì vậy đừng thêm ứng viên chỉ để thắng TRAIN.

Gợi ý: không phải núm nào "an toàn hơn" cũng an toàn hơn. Thế giới phản ứng lại policy của bạn.

### F · Xem trong 3D

`run_all.py export` ghi `evidence/viewer.json`. Mở `viewer/index.html`, sang chương **9 · phòng lab**, thả file vào:

- mỗi chấm là một kịch bản bạn đã thử, đậm dần theo vòng săn; bấm một chấm để chạy kịch bản đó;
- danh sách bên dưới là các va chạm bạn lưu theo từng kiểu;
- **bóng mờ** là xe ta với policy gốc trên cùng kịch bản, để thấy bản sửa của bạn thay đổi gì.

Chụp màn hình cho báo cáo (câu 5, câu 6), như ảnh dưới: kéo thanh thời gian đến đúng lúc va chạm (hoặc lúc chặn lối) rồi mới chụp.

![Chương 9: kiểu "chặn lối người đi bộ": người đi bộ bước ra từ sau xe buýt đỗ; xe ta phanh gấp và dừng, nhưng thân xe đã nằm trên vạch qua đường, người phải đứng lại](images/ch9-lab-crosswalk-block.jpg)

### G · Rắc rối của bạn (xưởng kịch bản, chương 10)

Các cuộc săn ở bước C chỉ tìm trong những gì thế giới đã có. Bước này bạn thêm vào thế giới một rắc rối mới.

1. **Brainstorm** (10 phút, có thể theo nhóm 2–3 người, nhưng mỗi người nộp file riêng). Ngoài đời, xe tự lái gặp chuyện gì mà thế giới này chưa có? Xem chương 10 có gì:
   - **người băng ngang** ở chỗ không có vạch;
   - **xe chết máy** đứng trong làn;
   - **phanh gấp**: một xe phía trước bất ngờ phanh;
   - **xe đạp lạng ra** từ mép đường.

   Mỗi rắc rối có thể đợi **đến khi xe ta còn cách N mét** mới xảy ra. Chọn cái bạn tin planner xử lý kém, và nói vì sao.
2. **Dựng** nó trong chương 10. Thử đến khi planner va chạm hoặc suýt va chạm. Bấm **"Bạn"** để tự lái và so với planner (nhớ rằng bạn đã xem lại cảnh nhiều lần, planner thì không). **Tải hazard.json** (hoặc Sao chép) vào thư mục lab.
   ![Chương 10, xưởng kịch bản: bốn nút thêm rắc rối, và thẻ H0 (một xe đạp lạng ra) đang chỉnh bằng thanh trượt](images/ch10-studio.jpg)

3. **Mở `hazard.json` (file tải từ chương 10: đã có sẵn `format`, `params`, `policy`, `hazards`; đừng xoá) và thêm ba trường bằng tay:**

   ```json
   "space": {"h0_x": [-150, 250], "h0_swerve": [0.6, 1.8], "h0_trig": [10, 40]},
   "exposure": 0.05,
   "why": "xe đạp lạng tránh ổ gà: ... (nguồn cho tần suất và độ lạng)"
   ```

   - `space`: mỗi con số của rắc rối thay đổi trong khoảng nào. `h0_x` là trường `x` của rắc rối H0. Chỉ được dùng khoá `h<i>_<trường số>` của rắc rối có thật, trong giới hạn `avsim/studio.py: LIMITS`; sai thì `mine` dừng và nói khoá nào sai.
   - `exposure`: rắc rối này xảy ra **bao nhiêu lần mỗi chuyến đi**.
   - `why`: căn cứ cho hai con số trên.

   Thêm `"world": "scene"` nếu muốn giữ nguyên thế giới bạn dựng thay vì giao thông của thị trấn. Khi đó mọi con số là **có điều kiện trên đúng cảnh đó**, không phải trên thị trấn của bạn (`mine.json: conditional_on`), và không nhân với `exposure` để so với lách xe buýt được.
4. `python run_all.py mine hazard.json`. Lệnh này:
   - chạy lại đúng cảnh bạn dựng;
   - quét khoảng 1 500 phiên bản trong giao thông của thị trấn bạn, bỏ những phiên bản bất khả thi (`accepted` = tỉ lệ giữ lại; xe đạp lạng gắt hơn 3 m/s² ngang hay phanh quá μg là bất khả thi);
   - đếm va chạm với policy gốc và với policy bạn chọn ở bước E;
   - phân mỗi va chạm vào một trong ba loại:
     - **tránh được bằng phanh hết cỡ ngay khi thấy** (`avoidable`): cùng planner, nhưng từ khung hình đầu tiên rắc rối đã bắt đầu **và** cảm biến xe ta nhìn thấy nó thì phanh hết cỡ, và không va chạm;
     - **không tránh được** (`unavoidable`): phanh hết cỡ ngay lúc đó vẫn va chạm;
     - **không quy được** (`unattributed`): chưa rắc rối nào bắt đầu khi va chạm xảy ra, tức một va chạm thường của thị trấn, không nói gì về rắc rối của bạn.

     Ngoài ra `low_speed` đếm những va chạm khi xe ta dưới 1 m/s (bị đâm khi đang đứng chờ). Chúng vẫn nằm trong loại của mình; báo riêng vì đó là chuyện khác với "xe ta đâm vào".

   Chỉ va chạm `avoidable` mới tính là lỗi của planner. `avoidable_foreknown` đếm thêm trường hợp phanh từ lúc rắc rối bắt đầu dù còn bị che: khoảng cách giữa hai con số là cái giá của việc bị che tầm nhìn. Phanh là cách tránh duy nhất được thử (không đánh lái). Nhân với `exposure` ra tần suất mỗi chuyến, đặt cạnh tần suất lách xe buýt của bạn.

Một rắc rối hay là rắc rối **tránh được** mà planner vẫn dính, và **bản sửa ở bước E không cứu**. Một rắc rối "100 % va chạm" mà không ai tránh nổi thì không chứng minh gì về planner.

## Nộp bài

`lab11/*.py`, `hazard.json`, `evidence/*.json`, `REPORT.md` (9 câu, trích số từ `evidence/` của bạn), ảnh chụp trong `evidence/shots/`. Grader chạy lại toàn bộ với MSSV của bạn trên bản sạch của các file không phải TODO, nên số trong báo cáo phải khớp với lần chạy lại. Xem `RUBRIC.md`.

## Không được

- Chạy kịch bản bất khả thi và đếm nó là thất bại.
- Báo một xác suất bằng 0 vì chưa thấy va chạm nào.
- Chọn policy trên TEST rồi báo kết quả trên chính TEST đó.
- Sửa `avsim/` hoặc phân phối tự nhiên để có con số đẹp hơn: grader dùng bản sạch.
- Đặt `exposure` hay `space` không có căn cứ để rắc rối của bạn trông nguy hiểm hơn.
