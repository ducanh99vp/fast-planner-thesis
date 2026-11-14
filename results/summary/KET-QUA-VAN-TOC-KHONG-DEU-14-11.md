# Kết quả: vật cản động vận tốc không đều (13–14/11)

## Kịch bản

- Mỗi vật cản tự đổi tốc độ sau một khoảng ngẫu nhiên 1–3 s; tốc độ mới lấy đều trong 0,3–1,3 m/s (trung bình 0,8); gia tốc không quá 1 m/s².
- Hướng vẫn dọc trục x hoặc y; chạm vật cản tĩnh, vật cản khác hoặc biên thì bật thẳng lại.
- Trong nhà: 8 tuyến (I1, I3, I3T, I3T5, I4D-R1, I4D-R2, I4D-R3, I5D). Ngoài trời: OUT, 24 vật cản.
- AN TOÀN = tới đích, và cách vật cản động ≥ 0,267 m, và cách tường ≥ 0,267 m.

## Các cấu hình

| Ký hiệu | Cấu hình | Nội dung |
|---|---|---|
| K | TN2P3LDMGT3HC28K | Bản cuối (commit 2b742c5), dự đoán vật cản đi thẳng đều |
| KV | TN2P3LDMGT3HC28KV | K + hướng 4: vật cản đổi vận tốc quá 0,15 m/s thì lập lại ngay (ĐÃ BỎ) |
| KU | TN2P3LDMGT3HC28KU | K + vùng bất định: vật cản dự đoán kéo dài dọc hướng chuyển động, mỗi phía e(τ) = min(½·a·τ², trần), a = 1,0 m/s², trần 0,5 m |

## Kết quả an toàn

| Đợt | Cấu hình | Vận tốc vật cản | Hạt | Trong nhà | Ngoài trời | Tổng |
|---|---|---|---|---|---|---|
| f7 | K | đều | 1–10 / 1–40 | 80/80 | 40/40 | 120/120 |
| f8 + f9 | K | đều | 11–20 / 41–80 | 80/80 | 40/40 | 120/120 |
| k1 | K | không đều | 1–10 / 1–40 | 78/80 | 36/40 | 114/120 |
| k2 | KV | không đều | 1–10 / 1–40 | 78/80 | 35/40 | 113/120 |
| k3 | KU | không đều | 1–10 / 1–40 | 80/80 | 40/40 | 120/120 |
| k4 | KU | không đều | 11–20 / 41–80 (hạt mới) | 80/80 | 40/40 | 120/120 |
| k5 | KU | đều | 1–10 / 1–40 | 80/80 | 40/40 | 120/120 |

- k1 so với f7 (cùng hạt): Fisher p = 0,029. Vận tốc không đều làm bản cuối kém đi.
- k2 so với k1: p = 1,0. Hướng 4 không cải thiện.
- k3 so với k1 (cùng hạt, cùng kịch bản): p = 0,029.
- KU gộp k3 + k4: 240/240, khoảng tin cậy 95 % (Clopper-Pearson) 98,5–100 %.
- k5 so với f7 (cùng hạt, cùng vận tốc đều): 120/120 so với 120/120, p = 1,0. Vùng bất định không làm mất an toàn ở vận tốc đều.
- KU gộp cả hai kịch bản (k3 + k4 + k5): 360/360, khoảng tin cậy 95 % là 99,0–100 %.

## Khoảng cách nhỏ nhất (m)

| Đợt | Vật cản động, trong nhà | Vật cản động, ngoài trời | Tường |
|---|---|---|---|
| k1 (K) | 0,259 | 0,071 | 0,262 |
| k2 (KV) | 0,191 | −0,222 | 0,331 |
| k3 (KU) | 0,337 | 0,366 | 0,304 |
| k4 (KU) | 0,288 | 0,389 | 0,301 |
| k5 (KU, vận tốc đều) | 0,362 | 0,438 | 0,342 |
| f7 (K, vận tốc đều) | 0,272 | 0,271 | 0,310 |

## Cái giá của vùng bất định

So theo từng cặp cùng hạt (Wilcoxon), chênh lệch trung vị:

| So sánh | η | T_f |
|---|---|---|
| k3 (KU) so với k1 (K), cùng không đều | +0,007 (p 0,013) | +0,24 s (p 0,004) |
| k3 (KU, không đều) so với f7 (K, đều) | +0,012 (p < 0,001) | +0,54 s (p < 0,001) |
| k4 (KU, không đều) so với f8 + f9 (K, đều) | +0,010 (p < 0,001) | +0,54 s (p < 0,001) |
| k5 (KU, đều) so với f7 (K, đều) | +0,009 (p < 0,001) | +0,53 s (p < 0,001) |

η trung vị của KU: trong nhà 1,014 (k3), 1,020 (k4), 1,012 (k5); ngoài trời 1,030 (k3), 1,029 (k4), 1,025 (k5).
Ở vận tốc đều (k5), T_f trung bình trong nhà 21,33 s (f7: 20,66 s), ngoài trời 32,18 s (f7: 31,03 s).
Riêng ngoài trời, T_f tăng khoảng 1,0 s (k3: p 0,01; k4: p 0,037).

## Ghi chú và hạn chế

- Nguyên nhân 13 lượt hỏng của k1 và k2: 9 lượt do vật cản đổi tốc trong khoảng 1 s cuối trước lúc gặp UAV (cả nhanh lên lẫn chậm lại), sai số dự đoán 0,24–0,47 m trong khi dự trữ chỉ 0,08 m; 3 lượt do biên vô hình; 1 lượt (k1, I4D-R2 hạt 3) do tường.
- a = 1,0 m/s² trùng giới hạn gia tốc cài trong bộ mô phỏng: phải khai báo giả định "UAV biết giới hạn gia tốc của vật cản".
- Trần 0,5 m chọn từ dữ liệu k1; k4 là phép thử trên hạt độc lập.
- k4: OUT hạt 43 nghiêng 26,5° (vẫn an toàn); A* thất bại 11 lần ở 5 lượt ngoài trời (k3: 1 lần).
- 14/11 chọn cách B: MỘT cấu hình (KU) cho cả vận tốc đều và không đều; k5 là phép đo KU ở vận tốc đều.
- k5: OUT hạt 32 nghiêng 30,9° (vẫn an toàn, 0,51 m), chưa phân tích.
- KU ở vận tốc đều mới đo trên hạt 1–10 / 1–40 (chưa đo hạt 11–20 / 41–80). CHƯA thử vật cản đổi hướng bất kỳ.
- Mã của ngày 13–14/11 chưa commit.

## Dữ liệu

- CSV: `results/raw/cd2_in_<đợt>_<tuyến>_<cấu hình>_<ngày>.csv`, `results/raw/cd2_out_<đợt>_<cấu hình>_<ngày>.csv` (k1, k2, k3: 20261113; k4, k5: 20261114).
- Bag và log: `~/cd2_chan_doan_16-10/bag_b1/`, `~/cd2_chan_doan_16-10/runs/`.
- So sánh: `~/cd2_chan_doan_16-10/scripts/so_sanh_dot.py`; phân tích từng lượt: `phan_tich_kd.py`.
