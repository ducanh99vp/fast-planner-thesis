#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
do_khoang_cach_dong.py — Mốc M4, luận văn UAV tự hành vòng tránh vật cản

Đo khoảng cách từ UAV tới vật cản động trong lúc bay, đếm số lần va chạm.
Dùng làm số liệu nền "trước khi có f_d" để so sánh ở M5.

Cách đo:
  - Vật cản động: /dynamic/obj (Marker, 30 lần/giây) cho tâm và kích thước hộp.
    Hộp không xoay (yaw = 0) nên là hộp song song trục toạ độ.
  - UAV: /visual_slam/odom, cùng nguồn với metric_logger.
  - Mỗi mẫu odom: khoảng cách từ tâm UAV tới mặt hộp gần nhất
    (bằng 0 nếu tâm UAV nằm trong hộp).
  - Chỉ tính khi UAV đang bay (z > 0.5 m).
  - Va chạm: khoảng cách < uav_radius. Một lần va chạm kéo dài tới khi
    khoảng cách vượt uav_radius + 0.05 m (tránh đếm lặp khi dao động quanh ngưỡng).
  - Xuyên qua: tâm UAV lọt hẳn vào trong hộp.

Chỉ đọc topic, không phát gì, không ảnh hưởng planner.

Cách dùng (sau khi đã chạy kino_replan.launch với dyn_obs:=1):
  python3 do_khoang_cach_dong.py --duration 300 --csv ~/m4_kc_dong.csv
"""

import argparse
import csv
import os
import threading

import numpy as np
import rospy
from nav_msgs.msg import Odometry
from visualization_msgs.msg import Marker

Z_FLY = 0.5      # m, thấp hơn mức này coi như đang ở mặt đất
HYST = 0.05      # m, trễ để kết thúc một lần va chạm


def dist_to_box(p, center, half):
    """Khoảng cách từ điểm p tới hộp song song trục (0 nếu p nằm trong hộp)."""
    q = np.abs(p - center) - half
    return float(np.linalg.norm(np.maximum(q, 0.0)))


class DynDistance:
    """Phần tính toán, tách khỏi ROS để thử được bằng dữ liệu giả."""

    def __init__(self, uav_radius):
        self.r = uav_radius
        self.boxes = {}          # id -> (tâm, nửa kích thước)
        self.rows = []           # (t, x, y, z, d, id gần nhất)
        self.in_contact = False
        self.n_hit = 0           # số lần va chạm
        self.n_through = 0       # số lần va chạm mà tâm UAV lọt vào trong hộp
        self.through_counted = False
        self.t_hit = 0.0         # tổng thời gian đang va chạm
        self.t_fly = 0.0         # tổng thời gian bay
        self.t_last = None
        self.lock = threading.Lock()

    def add_box(self, i, center, scale):
        with self.lock:
            self.boxes[i] = (np.asarray(center, float), 0.5 * np.asarray(scale, float))

    def add_uav(self, t, p):
        with self.lock:
            p = np.asarray(p, float)
            dt = 0.0 if self.t_last is None else min(t - self.t_last, 0.1)
            self.t_last = t
            if p[2] < Z_FLY or not self.boxes:
                return
            ds = {i: dist_to_box(p, c, h) for i, (c, h) in self.boxes.items()}
            i_min = min(ds, key=ds.get)
            d = ds[i_min]
            self.rows.append((t, p[0], p[1], p[2], d, i_min))
            self.t_fly += dt

            if d < self.r:
                self.t_hit += dt
                if not self.in_contact:
                    self.n_hit += 1
                    self.in_contact = True
                    self.through_counted = False
            elif d > self.r + HYST:
                self.in_contact = False

            # mỗi lần va chạm chỉ đếm xuyên qua một lần, kể cả khi mép hộp
            # (cập nhật 30 Hz) làm tâm UAV ra vào hộp vài mẫu liên tiếp
            if d == 0.0 and not self.through_counted:
                self.n_through += 1
                self.through_counted = True

    def summary(self):
        if not self.rows:
            return ["Chua co mau nao khi UAV dang bay."]
        d = np.array([r[4] for r in self.rows])
        return [
            "thoi gian bay:              %7.1f s  (%d mau)" % (self.t_fly, len(d)),
            "khoang cach nho nhat:       %7.3f m" % d.min(),
            "p5 khoang cach:             %7.3f m" % np.percentile(d, 5),
            "so lan va cham (d < %.2f):  %7d" % (self.r, self.n_hit),
            "  trong do xuyen qua hop:   %7d" % self.n_through,
            "tong thoi gian va cham:     %7.1f s  (%.1f %% thoi gian bay)" % (
                self.t_hit, 100.0 * self.t_hit / max(self.t_fly, 1e-9)),
        ]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--duration", type=float, default=300.0, help="giây đo")
    ap.add_argument("--uav_radius", type=float, default=0.2, help="m, giống metric_logger")
    ap.add_argument("--csv", default="", help="ghi từng mẫu ra file (tuỳ chọn)")
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node("do_khoang_cach_dong", anonymous=True)
    dd = DynDistance(args.uav_radius)

    def on_marker(msg):
        p = msg.pose.position
        dd.add_box(msg.id, (p.x, p.y, p.z), (msg.scale.x, msg.scale.y, msg.scale.z))

    def on_odom(msg):
        p = msg.pose.pose.position
        dd.add_uav(msg.header.stamp.to_sec(), (p.x, p.y, p.z))

    rospy.Subscriber("/dynamic/obj", Marker, on_marker, queue_size=20)
    rospy.Subscriber("/visual_slam/odom", Odometry, on_odom, queue_size=50)
    rospy.loginfo("Do khoang cach UAV - vat can dong trong %.0f s", args.duration)

    t_start = rospy.get_time()
    rate = rospy.Rate(2)
    try:
        while not rospy.is_shutdown() and rospy.get_time() - t_start < args.duration:
            rate.sleep()
    except rospy.ROSInterruptException:
        pass  # Ctrl+C: vẫn in kết quả của phần đã đo

    with dd.lock:
        print("\n".join(dd.summary()))
        if args.csv:
            path = os.path.expanduser(args.csv)
            with open(path, "w", newline="") as f:
                w = csv.writer(f)
                w.writerow(["t", "x", "y", "z", "d_dong", "vat_can"])
                w.writerows(dd.rows)
            print("da ghi %d dong vao %s" % (len(dd.rows), path))


if __name__ == "__main__":
    main()
