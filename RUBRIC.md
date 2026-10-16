# Lab 11 · Scenario Forge — thang điểm (100)

Grader lấy bài của bạn, thay `avsim/`, `run_all.py`, `tests/` và `policy_candidates.json` bằng bản sạch (giữ ứng viên bạn **thêm**), rồi chạy lại `run_all.py all` với MSSV của bạn. Mọi điểm dưới đây tính trên **lần chạy lại** đó, không tính trên `evidence/` bạn nộp, trừ phần D kiểm tra hai bản có khớp nhau.

## A · 10 TODO đúng (30)

Bộ test ẩn có cùng dạng với `tests/test_todos.py` nhưng số khác. Mỗi TODO 3 điểm, chia đều theo số test của TODO đó.

| TODO | Hàm |
|---|---|
| 1, 2, 9 | `metrics.min_ttc`, `metrics.contact_type`, `metrics.progress_metrics` |
| 3 | `feasible.is_feasible` |
| 4, 6, 7 | `estimate.wilson_ci`, `estimate.log_weights`, `estimate.is_estimate` |
| 5 | `search.cem_update` |
| 8 | `modes.label_mode` |
| 10 | `policy.choose_policy` |

## B · Pipeline cho ra con số đúng (20)

Grader chạy pipeline **tham chiếu** (lời giải của giảng viên) trên đúng seed của bạn, rồi so lần chạy lại của bài bạn với nó. Đúng code thì ra đúng số, kể cả khi seed của bạn xui. Bảng giá trị thật của thị trấn (tổng điều tra ≥ 80 000 lần chạy) chỉ dùng để giảng viên kiểm định lab, không dùng để chấm.

| Điểm | Kiểm tra trên lần chạy lại |
|---|---|
| 3 | `replay.json`: số va chạm của "phát lại" và "phản ứng", theo loại tiếp xúc, bằng tham chiếu. |
| 3 | `mc.json`: k, n và khoảng Wilson bằng tham chiếu (sai số 1e-9). |
| 5 | `estimate.json`: trung vị `total` qua 3 seed trong khoảng ×1,5 quanh tham chiếu (4 điểm; cuộc săn hỗn loạn, nên code đúng nhưng làm tròn khác vẫn lệch vài chục %); `other` và `upper_zero_hits` bằng tham chiếu (1 điểm). |
| 4 | `hunt.json`: cuộc săn `bus_bypass` và cuộc săn `crosswalk_block` mỗi cuộc tìm được ít nhất một thất bại đúng kiểu của nó, như tham chiếu. |
| 5 | `fix.json`: chạy lại với **seed TEST bí mật**: policy được chọn bằng tham chiếu, hoặc có `fail_hi` trên TEST không lớn hơn của lựa chọn tham chiếu và quãng đường giảm không quá 5 %. |

## C · Báo cáo câu 1–8 (30)

| Câu | Điểm | Điểm đầy đủ khi |
|---|---|---|
| 1 phát lại | 4 | Số khớp; giải thích rằng xe phía sau không phanh vì log không biết planner phanh sớm hơn người lái. |
| 2 bất khả thi | 3 | Mỗi luật có căn cứ vật lý; chỉ ra CEM bị kéo về kịch bản bất khả thi vì chúng "nguy hiểm" nhất. |
| 3 Monte Carlo | 4 | Khoảng Wilson đúng; n cần ≈ 96/p; kiểu chưa thấy được báo bằng cận trên, không bằng 0. |
| 4 IS | 5 | Bảng đủ; giải thích được `top_share`; so ngân sách IS với Monte Carlo có số. |
| 5 kiểu thất bại | 4 | Ảnh 3D cho mỗi kiểu; chuỗi sự kiện đúng với những gì trình xem chiếu; nói được vì sao cần săn riêng từng kiểu. |
| 6 sửa planner | 4 | TRAIN và TEST tách bạch; có ảnh với bóng mờ; giải thích một núm "an toàn" phản tác dụng bằng phản ứng của thế giới. |
| 7 sim → thật | 4 | Ba giả định, mỗi cái có nguồn dữ liệu cụ thể và hướng sai lệch. |
| 8 giới hạn | 2 | Ba giới hạn cụ thể của **thế giới này**, không phải câu chung chung. |

## G · Rắc rối của bạn (10)

| Điểm | Kiểm tra |
|---|---|
| 2 | `hazard.json` mở được trong chương 10, cảnh khả thi (không có cảnh báo vượt giới hạn), và `mine.json: exact` trên lần chạy lại khớp đúng cảnh đó. |
| 3 | `space` và `exposure` có căn cứ trong `why`; `space` hợp lệ và `accepted` ≥ 50 %; `mine.json` có ít nhất một va chạm `avoidable` với policy gốc, hoặc báo cáo giải thích vì sao rắc rối này planner đã xử lý tốt (cũng là một kết quả). |
| 5 | Câu 9: so policy gốc với bản sửa bằng số; phân biệt `avoidable`, `unavoidable` và `unattributed`, và nói vì sao "tránh được" là chuẩn lạc quan; đề xuất cách sửa cụ thể (núm chỉnh, hoặc logic mới nếu không núm nào sửa được). |

Rắc rối giống hệt của bạn cùng lớp, hoặc chép nguyên ví dụ trong README (cùng loại, cùng khoảng `space`), chia đôi điểm phần G (sau khi áp trần).

Câu 9, "tránh được là chuẩn lạc quan": người phanh biết rắc rối ngay khi thấy và phanh tối đa, nên `avoidable` **đếm nhiều hơn** số va chạm một planner thật tránh được (cận trên của lỗi planner theo nghĩa này), còn chỉ thử phanh (không đánh lái) làm nó đếm ít đi. Câu trả lời đầy đủ nói được cả hai hướng.

## D · Trung thực (10)

| Điểm | Kiểm tra |
|---|---|
| 5 | `evidence/` bạn nộp khớp lần chạy lại (cùng seed, cùng số va chạm, cùng policy được chọn). |
| 5 | Số trong `REPORT.md` khớp `evidence/` bạn nộp (trừ 1 điểm mỗi số sai hoặc trích sai file; số từ lần chạy thêm ngoài `evidence/` phải ghi rõ là vậy). |

## Trần điểm

| Vi phạm | Tổng tối đa |
|---|---|
| Sửa `avsim/`, phân phối tự nhiên hoặc seed để có con số đẹp hơn | 40 |
| `fix` chọn policy bằng dữ liệu TEST | 60 |
| Báo xác suất 0 cho một kiểu chỉ vì chưa thấy va chạm | trừ 4 ở câu 3 và câu 4 |
| `evidence/` nộp được tạo bằng `--quick` | phần D = 0 |
| `exposure` hoặc `space` bịa (không có `why`, hoặc `why` không khớp con số) | phần G tối đa 2 |
