#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
do_sai_so_du_doan.py — Mốc M4, luận văn UAV tự hành vòng tránh vật cản

Đo sai số dự đoán vị trí vật cản động của ObjPredictor tại chân trời
eval_horizon (mặc định 1 s). Cổng M4: sai số trung bình < 0.5 m.

Cách đo:
  - ObjPredictor phát /dynamic/prediction (PoseArray) 10 lần/giây.
    header.stamp = thời điểm mà dự đoán hướng tới (lúc phát + eval_horizon),
    poses[i] = vị trí dự đoán của vật cản i, NaN nếu chưa dự đoán được.
  - obj_generator phát vị trí thật /dynamic/pose_i 30 lần/giây, KHÔNG có stamp
    -> lấy thời điểm nhận làm thời điểm đo, giống ObjHistory::poseCallback.
  - Khi vị trí thật đã đi qua thời điểm đích, nội suy tuyến tính vị trí thật
    tại đúng thời điểm đó rồi lấy khoảng cách Euclid tới vị trí dự đoán.
  - Để so sánh, tính luôn sai số của giả thiết "vật cản đứng yên"
    (dự đoán = vị trí thật lúc nhận được bản tin dự đoán).

Chỉ đọc topic, không phát gì, không ảnh hưởng planner.

Cách dùng (sau khi đã chạy kino_replan.launch với dyn_obs:=1):
  python3 do_sai_so_du_doan.py --duration 300 --csv ~/m4_du_doan.csv
"""

import argparse
import bisect
import csv
import os
import threading

import numpy as np
import rospy
from geometry_msgs.msg import PoseArray, PoseStamped

GAP_MAX = 0.3    # s, hai mẫu vị trí thật cách nhau hơn mức này coi như đồng hồ nhảy
WARMUP = 5.0     # s, bỏ qua đoạn đầu khi lịch sử của ObjPredictor chưa đủ


class Evaluator:
    """Phần tính toán, tách khỏi ROS để thử được bằng dữ liệu giả."""

    def __init__(self, n_obj):
        self.n = n_obj
        self.t = [[] for _ in range(n_obj)]    # thời điểm nhận vị trí thật
        self.p = [[] for _ in range(n_obj)]    # vị trí thật tương ứng
        self.pending = []                      # (t_nhan, t_dich, [vị trí dự đoán])
        self.rows = []                         # (t_dich, id, sai_so_cv, sai_so_dung_yen)
        self.n_nan = 0
        self.n_jump = 0
        self.jump_times = []
        self.horizons = []
        self.lock = threading.Lock()   # callback ROS và vòng chấm chạy ở hai luồng

    def add_truth(self, i, t, pos):
        with self.lock:
            if self.t[i] and t - self.t[i][-1] > GAP_MAX:
                if not self.jump_times or t - self.jump_times[-1] > 1.0:
                    self.n_jump += 1   # các vật cản cùng thấy một lần nhảy -> chỉ đếm 1
                self.jump_times.append(t)
            self.t[i].append(t)
            self.p[i].append(np.asarray(pos, dtype=float))

    def add_prediction(self, t_recv, t_target, preds):
        with self.lock:
            self.pending.append((t_recv, t_target, preds))
            self.horizons.append(t_target - t_recv)

    def _interp(self, i, t):
        """Vị trí thật của vật cản i tại thời điểm t; None nếu không nội suy được."""
        ts = self.t[i]
        k = bisect.bisect_left(ts, t)
        if k == 0 or k >= len(ts):
            return None
        t0, t1 = ts[k - 1], ts[k]
        if t1 - t0 > GAP_MAX:
            return None
        a = (t - t0) / (t1 - t0)
        return (1 - a) * self.p[i][k - 1] + a * self.p[i][k]

    def _crosses_jump(self, t0, t1):
        return any(t0 <= tj <= t1 + GAP_MAX for tj in self.jump_times)

    def process(self, t_start):
        with self.lock:
            self._process(t_start)

    def _process(self, t_start):
        """Chấm các dự đoán đã tới hạn; giữ lại những dự đoán chưa tới."""
        latest = min((ts[-1] for ts in self.t if ts), default=-1.0)
        keep = []
        for t_recv, t_target, preds in self.pending:
            if t_target > latest:
                keep.append((t_recv, t_target, preds))
                continue
            if t_recv < t_start + WARMUP or self._crosses_jump(t_recv, t_target):
                continue
            for i, q in enumerate(preds):
                if not np.all(np.isfinite(q)):
                    self.n_nan += 1
                    continue
                truth = self._interp(i, t_target)
                now = self._interp(i, t_recv)
                if truth is None or now is None:
                    continue
                self.rows.append((t_target, i,
                                  float(np.linalg.norm(q - truth)),
                                  float(np.linalg.norm(now - truth))))
        self.pending = keep

    def summary(self):
        lines = []
        if not self.rows:
            return ["Chua co du doan nao duoc cham."], None
        e = np.array([r[2] for r in self.rows])
        s = np.array([r[3] for r in self.rows])
        ids = np.array([r[1] for r in self.rows])

        def stat(x):
            return "%6d  %6.3f  %6.3f  %6.3f  %6.3f" % (
                len(x), x.mean(), np.median(x), np.percentile(x, 95), x.max())

        lines.append("chan troi do duoc: %.3f s (trung binh)" % np.mean(self.horizons))
        lines.append("")
        lines.append("                     n    mean  median     p95     max   [m]")
        for i in range(self.n):
            if np.any(ids == i):
                lines.append("vat can %d  CV   %s" % (i, stat(e[ids == i])))
        lines.append("TAT CA     CV   %s" % stat(e))
        lines.append("TAT CA  dung yen %s" % stat(s))
        lines.append("")
        lines.append("ty le sai so < 0.5 m: %.1f %%" % (100.0 * np.mean(e < 0.5)))
        lines.append("du doan NaN bo qua: %d   mat mau / nhay dong ho: %d lan" % (self.n_nan, self.n_jump))
        ok = e.mean() < 0.5
        lines.append("CONG M4 (mean < 0.5 m): %s" % ("DAT" if ok else "CHUA DAT"))
        return lines, ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0, help="giây đo")
    ap.add_argument("--csv", default="", help="ghi từng sai số ra file (tuỳ chọn)")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("do_sai_so_du_doan", anonymous=True)
    rospy.loginfo("Cho ban tin dau tien tren /dynamic/prediction ...")
    first = rospy.wait_for_message("/dynamic/prediction", PoseArray)
    n = len(first.poses)
    rospy.loginfo("ObjPredictor theo doi %d vat can, do trong %.0f s", n, args.duration)

    ev = Evaluator(n)
    t_start = rospy.get_time()

    def on_pose(msg, i):
        p = msg.pose.position
        ev.add_truth(i, rospy.get_time(), (p.x, p.y, p.z))

    def on_pred(msg):
        preds = [np.array([q.position.x, q.position.y, q.position.z]) for q in msg.poses]
        ev.add_prediction(rospy.get_time(), msg.header.stamp.to_sec(), preds)

    subs = [rospy.Subscriber("/dynamic/pose_%d" % i, PoseStamped, on_pose, i) for i in range(n)]
    subs.append(rospy.Subscriber("/dynamic/prediction", PoseArray, on_pred))

    rate = rospy.Rate(2)
    try:
        while not rospy.is_shutdown() and rospy.get_time() - t_start < args.duration:
            ev.process(t_start)
            rate.sleep()
    except rospy.ROSInterruptException:
        pass  # Ctrl+C: vẫn in kết quả của phần đã đo
    ev.process(t_start)

    lines, _ = ev.summary()
    print("\n".join(lines))

    if args.csv:
        path = os.path.expanduser(args.csv)
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["t_dich", "vat_can", "sai_so_cv", "sai_so_dung_yen"])
            w.writerows(ev.rows)
        print("da ghi %d dong vao %s" % (len(ev.rows), path))


if __name__ == "__main__":
    main()
