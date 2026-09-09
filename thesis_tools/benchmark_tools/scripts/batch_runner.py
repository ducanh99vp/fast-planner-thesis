#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
batch_runner.py — Mốc M2, luận văn UAV tự hành vòng tránh vật cản

Chạy hàng loạt lượt mô phỏng không cần can thiệp tay.
KHÔNG phải node ROS — đây là script thường, gọi roslaunch qua subprocess.

Nguyên tắc hoạt động:
  Với mỗi (bản đồ × cấu hình × lượt):
     1. Khởi chạy bench.launch trong một nhóm tiến trình riêng
     2. metric_logger tự phát đích, tự đo, tự ghi CSV rồi tự tắt
     3. Script theo dõi số dòng trong CSV; thấy tăng là biết lượt đã xong
     4. Diệt cả nhóm tiến trình, dọn sạch, sang lượt sau

Cách dùng:
    python3 batch_runner.py --maps M1 M3 --configs B0 --trials 20
    python3 batch_runner.py --maps M1 --configs B0 --trials 3 --dry-run
"""

import os
import sys
import csv
import time
import signal
import argparse
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
DEFAULT_CSV = os.path.expanduser(
    "~/fast_planner_ws/src/Fast-Planner/results/raw/bench.csv")

# Cặp (xuất phát, đích) cho từng bản đồ. Sửa cho khớp bản đồ thật của bạn.
# Đích luôn ở độ cao 1.0 m vì FSM ép như vậy.
GOALS = {
    "M1": [(19.0, 0.0)],
    "M2": [( 5.0,  0.0), ( 5.0,  3.0), (-5.0,  2.0), ( 4.0, -3.0)],
    "M3": [( 8.0,  8.0), ( 8.0, -6.0), (-7.0,  7.0)],
    "M4": [(10.0,  0.0), (10.0,  5.0), (-9.0,  4.0)],
    "M5": [( 8.0,  5.0), ( 8.0, -4.0), (-7.0,  5.0)],
}


def count_rows(path):
    if not os.path.isfile(path):
        return 0
    with open(path, newline="") as f:
        return max(0, sum(1 for _ in f) - 1)


def run_one(map_name, config, trial, goal, out_csv, timeout, extra):
    """Chạy một lượt. Trả về True nếu logger đã ghi được dòng."""
    before = count_rows(out_csv)

    cmd = [
        "roslaunch", "benchmark_tools", "bench.launch",
        "map_name:=%s" % map_name,
        "config:=%s" % config,
        "trial:=%d" % trial,
        "goal_x:=%.3f" % goal[0],
        "goal_y:=%.3f" % goal[1],
        "out_csv:=%s" % out_csv,
        "timeout:=%.1f" % timeout,
    ] + list(extra)

    print("  → %s" % " ".join(cmd[2:]))
    proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL,
                            preexec_fn=os.setsid)

    deadline = time.time() + timeout + 45.0   # cộng thêm thời gian khởi động
    ok = False
    try:
        while time.time() < deadline:
            if count_rows(out_csv) > before:
                ok = True
                time.sleep(2.0)      # chờ logger ghi xong hoàn toàn
                break
            if proc.poll() is not None:
                time.sleep(2.0)
                ok = count_rows(out_csv) > before
                break
            time.sleep(0.5)
    finally:
        try:
            os.killpg(os.getpgid(proc.pid), signal.SIGINT)
            proc.wait(timeout=10)
        except Exception:
            try:
                os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
            except Exception:
                pass
        subprocess.call(["pkill", "-f", "fast_planner_node"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-f", "quadrotor_simulator"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        subprocess.call(["pkill", "-f", "pcl_render_node"],
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        time.sleep(3.0)              # để cổng ROS được giải phóng
    return ok


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--maps", nargs="+", default=["M1"])
    ap.add_argument("--configs", nargs="+", default=["B0"])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--out", default=DEFAULT_CSV)
    ap.add_argument("--dry-run", action="store_true",
                    help="chỉ in ra lịch chạy, không thực thi")
    ap.add_argument("--extra", nargs="*", default=[],
                    help="tham số roslaunch bổ sung, ví dụ dynamic_env:=1")
    a = ap.parse_args()

    total = len(a.maps) * len(a.configs) * a.trials
    print("=" * 62)
    print("Tổng số lượt chạy: %d" % total)
    print("Ước tính thời gian: %.1f giờ (giả định %.0f giây mỗi lượt)"
          % (total * (a.timeout * 0.5 + 40) / 3600.0, a.timeout * 0.5 + 40))
    print("Ghi vào: %s" % a.out)
    print("=" * 62)

    if a.dry_run:
        for m in a.maps:
            for c in a.configs:
                for t in range(a.trials):
                    g = GOALS.get(m, [(5.0, 0.0)])[t % len(GOALS.get(m, [(5.0, 0.0)]))]
                    print("  %s | %s | lượt %2d | đích (%.1f, %.1f)" % (m, c, t, g[0], g[1]))
        return

    done, failed = 0, 0
    t0 = time.time()
    for m in a.maps:
        goals = GOALS.get(m, [(5.0, 0.0)])
        for c in a.configs:
            for t in range(a.trials):
                done += 1
                g = goals[t % len(goals)]
                print("[%d/%d] %s | %s | lượt %d" % (done, total, m, c, t))
                ok = run_one(m, c, t, g, a.out, a.timeout, a.extra)
                if not ok:
                    failed += 1
                    print("     KHÔNG ghi được dòng nào — xem lại lượt này")
                el = time.time() - t0
                print("     đã chạy %.1f phút, còn khoảng %.1f phút"
                      % (el / 60.0, el / done * (total - done) / 60.0))

    print("=" * 62)
    print("Xong. %d lượt, %d lượt không ghi được dữ liệu." % (total, failed))
    print("Tổng số dòng trong CSV: %d" % count_rows(a.out))


if __name__ == "__main__":
    main()
