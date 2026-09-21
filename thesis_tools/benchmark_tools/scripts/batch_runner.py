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
    python3 batch_runner.py --maps I1 I3 --configs B0 --trials 20
    python3 batch_runner.py --maps I1 --configs B0 --trials 3 --dry-run
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

# [Luan van - M3] Cau hinh tung ban do Indoor.
#   size : kich thuoc ban do planner = phong + 1 m theo x, y; z = 4.5 (chua tran 3 m)
#   start: diem xuat phat tren mat dat (UAV tu cat canh len takeoff_height)
#   goals: dich cach vat can > 0.5 m VA nam trong vung toi duoc tu start
MAPS_DIR = os.path.expanduser(
    "~/fast_planner_ws/src/Fast-Planner/uav_simulator/map_generator/maps")
MAPS = {
    "I1": dict(pcd="I1", size=(16.0, 11.0, 4.5), start=(-6.0, 0.0), goals=[(6.0, 0.0)]),
    "I3": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),
    # I4: mot ban do, ba tuyen bay qua ba o cua rong khac nhau.
    #   R1 qua cua A 1.4 m, R2 qua cua C 1.2 m, R3 qua cua D 0.9 m (hep nhat)
    "I4-R1": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(10.0, -5.0)]),
    "I4-R2": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(-10.0, 5.0)]),
    "I4-R3": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(10.0, -5.0), goals=[(10.0, 5.0)]),
    # I5: ngo cut chu U kin toi tran, mieng quay ve xuat phat — cuc tieu dia phuong
    "I5": dict(pcd="I5", size=(21.0, 16.0, 4.5), start=(-8.0, 0.0), goals=[(8.0, 0.0)])
}

# Tham so bay chung cho moi ban do
FLIGHT_ARGS = ["init_z:=0.0", "takeoff_height:=1.0", "max_vel:=1.5", "max_acc:=1.5"]


# [Luan van - M3] Hai bo tham so chay so sanh:
#   P0 — bo mac dinh cua Fast-Planner, hieu chinh cho ngoai troi (van dung cho CD3)
#   P1 — bo Indoor: o luoi nho hon de khong nuot mat cua hep, vanh dai an toan
#        phu duoc ban kinh than UAV, nguong phat lui lai de qua duoc cua 0.9 m
CONFIGS = {
    "P0": [],
    # local_range giu 5.5: thu 4.0 (theo lo trinh) thi nho hon search/horizon 4.5,
    # A* tim vao dai ban do khong duoc duy tri -> lap lai nhieu, j_rms R3 tang 29%
    "P1": ["map_resolution:=0.05", "inflation:=0.15", "dist0:=0.3",
           "virtual_ceil:=2.8"],
    # P2 — nen cho moi thi nghiem TRONG NHA: mac dinh, chi doi tran ao theo
    # tran that 3.0 m. Ngoai troi (CD3) dung P0 voi tran ao 2.5.
    "P2": ["virtual_ceil:=2.8"],


}


def map_args(map_name):
    """Tham so roslaunch rieng cua mot ban do Indoor."""
    m = MAPS[map_name]
    return ["use_pcd:=true",
            "map_file:=%s/%s.pcd" % (MAPS_DIR, m["pcd"]),
            "map_size_x:=%.1f" % m["size"][0],
            "map_size_y:=%.1f" % m["size"][1],
            "map_size_z:=%.1f" % m["size"][2],
            "init_x:=%.2f" % m["start"][0],
            "init_y:=%.2f" % m["start"][1]] + FLIGHT_ARGS


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
    ] + map_args(map_name) + CONFIGS[config] + list(extra)  # extra dat cuoi de ghi de duoc

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
    ap.add_argument("--maps", nargs="+", default=["I1"])
    ap.add_argument("--configs", nargs="+", default=["P0"])
    ap.add_argument("--trials", type=int, default=20)
    ap.add_argument("--timeout", type=float, default=120.0)
    ap.add_argument("--out", default=DEFAULT_CSV)
    ap.add_argument("--dry-run", action="store_true",
                    help="chỉ in ra lịch chạy, không thực thi")
    ap.add_argument("--extra", nargs="*", default=[],
                    help="tham số roslaunch bổ sung, ví dụ dynamic_env:=1")
    # [Luan van - M5] bat vat can dong; moi luot mot hat giong de tai lap duoc
    ap.add_argument("--dyn", type=int, default=0,
                    help="so vat can dong (0 = tat)")

    a = ap.parse_args()
    
    # [Luan van - M5] ghi sang CSV rieng, khong lan voi baseline tinh cua M3
    if a.dyn > 0 and a.out == DEFAULT_CSV:
        a.out = DEFAULT_CSV.replace("bench.csv", "bench_dyn.csv")
    
    for m in a.maps:
        if m not in MAPS:
            sys.exit("Ban do %s chua co trong MAPS" % m)
        pcd = "%s/%s.pcd" % (MAPS_DIR, MAPS[m]["pcd"])
        if not os.path.isfile(pcd):
            sys.exit("Khong thay file %s - hay sinh ban do truoc" % pcd)
    for c in a.configs:
        if c not in CONFIGS:
            sys.exit("Cau hinh %s chua co trong CONFIGS" % c)


    total = len(a.maps) * len(a.configs) * a.trials
    print("=" * 62)
    print("Tổng số lượt chạy: %d" % total)
    print("Ước tính thời gian: %.1f giờ (giả định %.0f giây mỗi lượt)"
          % (total * (a.timeout * 0.5 + 40) / 3600.0, a.timeout * 0.5 + 40))
    print("Vật cản động: %s" % ("%d vật cản, hạt giống = số lượt + 1" % a.dyn if a.dyn > 0 else "không"))
    print("Ghi vào: %s" % a.out)
    print("=" * 62)

    if a.dry_run:
        for m in a.maps:
            for c in a.configs:
                for t in range(a.trials):
                    g = MAPS[m]["goals"][t % len(MAPS[m]["goals"])]
                    print("  %s | %s | lượt %2d | đích (%.1f, %.1f)" % (m, c, t, g[0], g[1]))
                    print("     " + " ".join(map_args(m)))
        return

    done, failed = 0, 0
    t0 = time.time()
    for m in a.maps:
        goals = MAPS[m]["goals"]
        for c in a.configs:
            for t in range(a.trials):
                done += 1
                g = goals[t % len(goals)]
                print("[%d/%d] %s | %s | lượt %d" % (done, total, m, c, t))
                extra = list(a.extra)
                if a.dyn > 0:
                    # dat truoc a.extra de nguoi dung van ghi de duoc bang --extra
                    extra = ["dyn_obs:=1", "dyn_num:=%d" % a.dyn,
                             "dyn_seed:=%d" % (t + 1)] + extra
                ok = run_one(m, c, t, g, a.out, a.timeout, extra)
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
