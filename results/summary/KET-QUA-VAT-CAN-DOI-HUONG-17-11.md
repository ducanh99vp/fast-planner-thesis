# Kết quả: vật cản đổi hướng bất kỳ (16–17/11)

Bước 2 sau vận tốc không đều (xem `KET-QUA-VAN-TOC-KHONG-DEU-14-11.md`).

## Kịch bản

- Mỗi vật cản có đồng hồ riêng 1–3 s (ngẫu nhiên); hết giờ thì bốc hướng đích mới, đều trong 0–360°.
- Vật cản rẽ dần về hướng đích với gia tốc ngang không quá 1,0 m/s² (ở 0,8 m/s là 72°/s; quay đầu 180° mất 2,5 s).
- Tốc độ giữ đều 0,8 m/s (tách riêng ảnh hưởng của đổi hướng; chưa gộp với đổi tốc).
- Hướng ban đầu ngẫu nhiên. Chạm vật cản tĩnh thì quay ngược (v = −v); chạm vật cản động khác hoặc biên thì lật một thành phần vận tốc (luật cũ, khớp bộ dự đoán); sau mỗi lần bật lại, hướng đích = hướng mới. Hộp không tự xoay.
- Bộ sinh số riêng cho mỗi vật cản (hạt, chỉ số, 1611) nên vị trí sinh vật cản trùng các đợt trước.
- Chỉ ngoài trời: OUT, 24 vật cản, hạt 1–40.
- Bật bằng `dyn_var_dir:=true` (mặc định tắt; tham số phụ `dyn_dir_tmin`, `dyn_dir_tmax`, `dyn_dir_acc`).
- Kiểm 1 lượt (OUT hạt 16): tốc độ 0,800 m/s; tốc độ rẽ p99 72,1°/s; gia tốc ngang p99 1,01 m/s²; vật cản đi thẳng 53 % thời gian.
- AN TOÀN = tới đích, và cách vật cản động ≥ 0,267 m, và cách tường ≥ 0,267 m.

## Các cấu hình

| Ký hiệu | Cấu hình | Nội dung |
|---|---|---|
| KU | TN2P3LDMGT3HC28KU | Bản cuối của vận tốc đều + không đều (commit 9a15038): vùng bất định kéo dài dọc hướng chuyển động, e(τ) = min(½·a·τ², 0,5 m), a = 1,0 m/s² |
| KUT | TN2P3LDMGT3HC28KUT | KU + vùng bất định HÌNH TRÒN (`dyn_unc_iso:=true`): vật cản dự đoán nở đều mọi hướng nằm ngang thêm e(τ); trụ: bán kính + e; hộp: hộp bo góc; chiều cao không nở |
| **KUTN** | **TN2P3LDMGT3HC28KUTN** | **KUT + dự đoán HAI NHÁNH (`dyn_reflect_both:=true`): ở lần phản xạ đầu tiên vào vật cản tĩnh, giữ thêm một nhánh đi thẳng tiếp. CHỐT cho bước 2 (17/11)** |
| KUTN75 | TN2P3LDMGT3HC28KUTN75 | KUTN + trần vùng bất định 0,75 m thay 0,5 m. Thí nghiệm độ nhạy, không dùng |

Vùng bất định đặt quanh VỊ TRÍ DỰ ĐOÁN ở từng thời điểm tương lai (không phải quanh vị trí hiện tại); τ = thời gian kể từ lần quan sát mới nhất. Dùng chung cho A*, tối ưu, hậu kiểm P3H và phép kiểm 20 Hz.

## Kết quả an toàn (ngoài trời, hạt 1–40, cùng hạt)

| Đợt | Cấu hình | Vật cản | An toàn | KTC 95 % (Clopper-Pearson) |
|---|---|---|---|---|
| k5 | KU | đi dọc trục, vận tốc đều (đối chứng) | 40/40 | 91,2–100 % |
| g1 | KU | đổi hướng | 28/40 | 53,5–83,4 % |
| g2 | KUT | đổi hướng | 37/40 | 79,6–98,4 % |
| **g3** | **KUTN** | **đổi hướng** | **39/40** | **86,8–99,9 %** |
| g4 | KUTN75 | đổi hướng | 39/40 | 86,8–99,9 % |

Cả 5 đợt: 40/40 tới đích.

Fisher:

- g1 so với k5: p = 0,0002. Vật cản đổi hướng làm bản cuối KU kém đi rõ.
- g2 so với g1: p = 0,020. Vùng tròn có tác dụng.
- g3 so với g2: p = 0,62. CHƯA có ý nghĩa: không kết luận được dự đoán hai nhánh giúp thêm.
- g3 so với g1: p = 0,0015. Hai phần sửa gộp lại có tác dụng rõ.
- g3 so với k5: p = 1,0.
- g4 so với g3: p = 1,0. Nâng trần không tăng an toàn.

## Khoảng cách nhỏ nhất (m)

| Đợt | Vật cản động | Tường |
|---|---|---|
| k5 (KU, dọc trục) | 0,438 | 0,342 |
| g1 (KU) | −0,296 | 0,334 |
| g2 (KUT) | 0,085 | 0,300 |
| g3 (KUTN) | 0,219 | 0,328 |
| g4 (KUTN75) | 0,117 | 0,307 |

## Cái giá

| Đợt | η trung vị | η trung bình | T_f trung bình | Lượt nghiêng > 25° | Nghiêng trung vị | Lập kế hoạch thất bại hẳn |
|---|---|---|---|---|---|---|
| k5 (KU, dọc trục) | 1,025 | 1,039 | 32,18 s | 1 | 9,2° | 0 |
| g1 (KU) | 1,048 | 1,075 | 34,71 s | 7 | 15,8° | 92 lần ở 6 lượt |
| g2 (KUT) | 1,115 | 1,126 | 35,93 s | 5 | 13,9° | 116 lần ở 6 lượt |
| g3 (KUTN) | 1,107 | 1,146 | 36,80 s | 1 | 12,9° | 23 lần ở 4 lượt |
| g4 (KUTN75) | 1,262 | 1,315 | 43,18 s | 12 | 16,5° | 295 lần ở 6 lượt |

"Lập kế hoạch thất bại hẳn" = cột `N_fail` = số lần lập lại không ra quỹ đạo sau MỌI lần thử lại (A* nới biên, khởi tạo gián đoạn). Không phải số lần A* thất bại ở lần thử đầu.

So theo từng cặp cùng hạt (Wilcoxon), chênh lệch trung vị:

| So sánh | η | T_f |
|---|---|---|
| g1 (KU) so với k5 (KU, dọc trục) | +0,029 (p 0,011) | +2,12 s (p 0,001) |
| g2 (KUT) so với g1 | +0,033 (p 0,0004) | +0,83 s (p 0,10) |
| g2 (KUT) so với k5 | +0,062 (p < 0,001) | +2,70 s (p < 0,001) |
| g3 (KUTN) so với g2 | +0,016 (p 0,34) | +1,03 s (p 0,28) |
| g3 (KUTN) so với g1 | +0,058 (p 0,0002) | +1,23 s (p 0,033) |
| **g3 (KUTN) so với k5** | **+0,093 (p < 0,001)** | **+3,65 s (p < 0,001)** |
| g4 (KUTN75) so với g3 | +0,164 (p < 0,001) | +7,0 s (p < 0,001) |
| g4 (KUTN75) so với k5 | +0,226 (p < 0,001) | +11,0 s (p < 0,001) |

Nghiêng > 25°: g4 12/40 so với g3 1/40, Fisher p = 0,0015 (g4: hạt 34 71,2°, hạt 33 67,9°, hạt 40 57,0°, hạt 24 50,8°). g3: chỉ hạt 35 (33,0°).

## Nguyên nhân các lượt hỏng

### g1 (KU): 12 lượt — hạt 12, 14, 19, 20, 24, 31, 32, 33, 36, 37, 38, 39

- 12/12 cùng nguyên nhân: vật cản ĐANG RẼ trong 1–2 s cuối trước lúc gặp UAV. Không lượt nào do bật lại.
- Hướng đổi trong 1 s cuối: 21–68°. Dự đoán đi thẳng từ 1 s trước lệch NGANG 0,36–0,48 m, lệch DỌC chỉ −0,10…−0,22 m: vùng dọc của KU không phủ phương ngang.
- Planner có phản ứng (cảnh báo đầu tiên 0,6–3,4 s trước, lập lại 3–21 lần), nhưng mỗi kế hoạch mới lại giả định vật cản đi thẳng.
- Hạt 12 thêm vấn đề thứ hai: sau lần chạm đầu, A* thất bại 73 lần trong 4 s, UAV kẹt cạnh vật cản 3,5 s → −0,296 m.

### g2 (KUT): 3 lượt — ba cơ chế khác nhau

- Hạt 28 (0,140 m): bộ dự đoán báo vật cản sẽ chạm cột rồi phản xạ, nhưng vật cản đang rẽ nên không chạm. Planner tưởng vật cản quay đi, không cảnh báo tới 0,56 s trước; không đủ để né. → Lý do thêm dự đoán hai nhánh (KUTN).
- Hạt 1 (0,195 m): sai số dự đoán ở 1,5–2 s (0,86–1,18 m) vượt trần 0,5 m. UAV giảm còn 0,4 m/s ngay trước mũi vật cản; kế hoạch tốt nhất chỉ còn 0,10–0,21 m.
- Hạt 27 (0,085 m): vật cản rẽ rồi chạm cột thật, bật về phía UAV đang ở sát cột (0,36–0,6 m). A* thất bại khoảng 100 lần trong 1,3 s, UAV bay quỹ đạo cũ. Cùng họ với g1 hạt 12.

### g3 (KUTN): 1 lượt — hạt 19 (0,219 m, thân UAV lấn vào vật cản 4,8 cm)

Cùng cơ chế với g2 hạt 1: sai số dự đoán xa hơn 1 s vượt trần 0,5 m.

1. Vật cản 3 (trụ, bán kính 0,25 m) rẽ liên tục 134° trong 1,75 s ở tốc độ rẽ tối đa (~72°/s), từ hướng tây nam (xa đường bay) vòng lên bắc, cắt ngang hành lang UAV.
2. Bộ dự đoán báo vật cản sẽ chạm vật cản tĩnh rồi nảy; nhánh phụ giữ khả năng đi thẳng. Cả hai nhánh đều đưa vật cản ra xa UAV; thực tế nó không nảy cũng không đi thẳng mà rẽ (tới cách điểm dự đoán chạm ~0,17 m rồi rẽ đi).
3. Dự đoán đi thẳng lệch 0,55 m sau 1 s, 1,08 m sau 1,5 s, 1,50 m sau 2 s; vòng tròn dừng ở 0,5 m.
4. Phép kiểm 20 Hz (nhìn 1,5 s, ngưỡng 0,3 m) báo một lần ở 2,33 s trước, rồi gần 1 s không báo; báo liên tục từ 1,33 s trước (cách ~1,6 m). 15 lần lập lại trong 1,3 s cuối:
   - 5 kế hoạch đầu (1,27–0,86 s trước) vẫn dựa trên dự đoán sai vì vật cản đang rẽ: lúc lập planner thấy cách 0,30–0,38 m, nếu bay hết thì chỉ 0,16 đến −0,16 m. UAV giảm tốc 1,49 → 0,59 m/s.
   - Từ 0,77 s trước, dự đoán ngắn hạn khớp thực tế, nhưng với gia tốc tối đa 1,5 m/s² kế hoạch tốt nhất chỉ còn 0,29 → 0,19 m.
5. Đã loại trừ: không có lần lập kế hoạch nào thất bại hẳn (A* thất bại 6 lần ở lần thử đầu, đều được thử lại cứu); vật cản 17 ở phía bắc không chặn (kế hoạch cách nó 0,79–0,93 m); không liên quan tường.

Dựng lại offline trên đúng các kế hoạch đã bay: với trần 0,75 m, phép kiểm báo từ 2,60 s trước (cách 4,1 m) thay vì 1,35 s (1,8 m) → lý do chạy g4.

### g4 (KUTN75): 1 lượt — hạt 33 (0,117 m)

Lập kế hoạch thất bại hẳn 287 lần, nghiêng 67,9°. CHƯA phân tích (nghi UAV bị kẹt, chưa kiểm). Hạt 19 ở g4 an toàn (0,75 m), nhưng cùng hạt không cho lại cùng kết quả nên không chứng minh được gì.

## Ghi chú và hạn chế

- Chốt KUTN (trần 0,5 m) cho bước 2. Lượt kiểu hạt 19 là hạn chế được ghi nhận: khi vật cản rẽ gắt lâu hơn 1 s đúng lúc đang tiến về phía UAV, sai số dự đoán vượt trần 0,5 m.
- ½·a·τ² tự nó là cận trên đúng của sai số khi gia tốc ngang ≤ a (rẽ hết cỡ: lệch 0,48 / 1,02 / 1,68 m sau 1 / 1,5 / 2 s, so với ½·a·τ² = 0,50 / 1,12 / 2,00 m). Chỉ có trần làm vùng hụt khi nhìn xa hơn 1 s. Bỏ trần thì ở 2 s mỗi vật cản trụ rộng ~4,5 m, với 24 vật cản UAV gần như không còn đường; trần 0,75 m (g4) đã không tăng an toàn mà làm đường dài thêm ~16 % và 12/40 lượt nghiêng > 25°.
- Lập lại kế hoạch dày (~0,1 s/lần, ~15 ms tính toán) không bù được: mỗi lần lập lại chỉ biết hướng đi hiện tại, không biết vật cản đang rẽ, nên mọi dự đoán mới đều trễ sau cú rẽ về cùng một phía.
- Từ 39/40, một đợt 40 hạt không thể chứng minh cải thiện an toàn (kể cả 40/40 cũng p = 1,0).
- Mỗi cấu hình mới chỉ đo một đợt 40 hạt trên một bộ hạt; chưa đo hạt độc lập, chưa đo trong nhà, chưa gộp đổi hướng với đổi tốc (khi gộp: gia tốc dọc và ngang chặn riêng, tổng có thể tới √2 m/s²).
- a = 1,0 m/s² trùng giới hạn gia tốc ngang của bộ sinh vật cản: giả định "UAV biết giới hạn gia tốc của vật cản".
- Gia tốc tối đa của UAV giữ 1,5 m/s² như mọi thí nghiệm từ M3; tăng lên 2,0 chỉ thêm tối đa ~0,15 m ở 0,77 s cuối (cận trên thô) và đổi mô hình UAV → không thử.
- Hướng phát triển (chưa thử): dự đoán theo tốc độ quay (đúng khi vật cản đang rẽ, sai khi nó thôi rẽ).
- Chưa phân tích: g4 hạt 33; các lượt nghiêng > 25° (g1 hạt 10: 65,7°, hạt 20: 55,6°; g2 hạt 31: 45,7°; g3 hạt 35: 33,0°).
- Tất cả tham số mới (`dyn_var_dir`, `dyn_unc_iso`, `dyn_reflect_both`) mặc định TẮT: các kịch bản và cấu hình cũ không đổi.
- Số liệu thô: `results/raw/cd2_out_{g1,g2,g3,g4}_*.csv` (không đưa vào git); bag và log ở `~/cd2_chan_doan_16-10/`.
