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
    "I3G": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),  # [CD2] can gat nuoc
    # I4: mot ban do, ba tuyen bay qua ba o cua rong khac nhau.
    #   R1 qua cua A 1.4 m, R2 qua cua C 1.2 m, R3 qua cua D 0.9 m (hep nhat)
    "I4-R1": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(10.0, -5.0)]),
    "I4-R2": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(-10.0, 5.0)]),
    "I4-R3": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(10.0, -5.0), goals=[(10.0, 5.0)]),
    # I5: ngo cut chu U kin toi tran, mieng quay ve xuat phat — cuc tieu dia phuong
    "I5": dict(pcd="I5", size=(21.0, 16.0, 4.5), start=(-8.0, 0.0), goals=[(8.0, 0.0)]),
    # [CD2] Ban do NGOAI TROI cua HKUST (random_forest). pcd=None => use_pcd:=false.
    #   60 cot vuong CAO 3 m (khong bay vuot duoc) + 10 vanh tron.
    #   Vat can dong don vao hanh lang |y| <= 3 m quanh duong bay y = 0
    #   (lan thu voi |y| <= 8: UAV mu van sach 80% luot -> qua thua).
    "OUT": dict(pcd=None, size=(40.0, 20.0, 5.0), start=(-19.0, 0.0),
                goals=[(19.0, 0.0)],
                # virtual_floor -0.05: ban do HKUST KHONG co mat dat, vung z in [-1, 0)
                # trong tron -> P2 ep UAV chui xuong duoi vat can (lenh z toi -1.73 m)
                extra=["p_num:=60", "c_num:=10", "p_hmin:=3.0", "dyn_xy_y:=3.0",
                       "virtual_floor:=-0.05",
                       # cat canh 0.5 m (nguoi dung chot 12/10/2026); giua chung UAV bay TU DO
                       # trong [san ao, tran ao 2.5]. FSM goc ep z dich = 1.0 -> goal_z
                       "takeoff_height:=0.5", "goal_z:=0.5"])
}

# [CD2 Indoor] Vat can dong trong nha (chi dung khi --dyn != 0; --dyn -1 = so luong mac dinh
# duoi day). Vung sinh "x0,x1,y0,y1;..." tranh 1.5 m quanh xuat phat / dich va tranh
# phan bi bit kin (I3). xy = bien phan xa (nua kich thuoc phong). z_margin 0.15: bo qua
# lop sat san / tran khi xet va cham tinh (neu khong, vat can luon "cham" san).
DYN_INDOOR = {
    "I1":    dict(num=4, xy=(7.5, 5.0),  spawn="-4.5,4.5,-4.5,4.5"),
    # I3: vat can di DOC hanh lang (nhanh ngang theo x, nhanh doc theo y). Di ngang hanh lang
    # thi bat qua lai giua hai vach ~2.5 s/lan nhu can gat nuoc — giu lai lam kich ban kho rieng I3G.
    "I3":    dict(num=4, xy=(10.0, 10.0), spawn="-6.5,6.5,-9.6,-7.4,0;7.4,9.6,-6.5,6.5,1"),
    "I3G":   dict(num=4, xy=(10.0, 10.0), spawn="-6.5,6.5,-9.6,-7.4;7.4,9.6,-6.5,6.5"),
    "I4-R1": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    "I4-R2": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    "I4-R3": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    "I5":    dict(num=4, xy=(10.0, 7.5), spawn="-6,1,4.8,7;-6,1,-7,-4.8;3,6.5,-7,7"),
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
    # [CD2] Thang bac ne vat can dong. Ten SWxx de khong trung "P0/P1/P2" ben tren
    # (do la BO THAM SO cua M3, khong phai phuong an ne). SW = ghi VET QUET cua
    # vat can dong trong xx/10 giay toi vao ban do chiem dung.
    # SW00: chi ve vi tri HIEN TAI cua vat can dong, khong du doan -> tach phan
    # dong gop cua "nhin thay" khoi phan dong gop cua "du doan" (thi nghiem A)
    "SW00": ["dyn_sweep_T:=0.0"],
    "SW05": ["dyn_sweep_T:=0.5"],
    "SW10": ["dyn_sweep_T:=1.0"],
    "SW15": ["dyn_sweep_T:=1.5"],
    "SW25": ["dyn_sweep_T:=2.5"],
    # SW15 + bong bong thoat 0.6 m cua A*: bo qua o CHI do vet quet quanh diem
    # xuat phat. Truoc khi co: 623 lan tim duong that bai / 10 hat, 99% do diem
    # xuat phat nam trong vet quet.
    "SW15E": ["dyn_sweep_T:=1.5", "start_escape:=0.6"],
    # SW15 + vung phinh vat can TINH 0.2 m (= ban kinh UAV; mac dinh 0.099). Vet quet giu
    # 0.099 de chi mot thu thay doi. Ly do: hat 35 suot cot o 5 cau hinh (d 0.14-0.18 m).
    "SW15I2": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.099"],
    # === P2 HOAN CHINH (chot 13/10/2026) === SW15I2 + bong bong thoat 0.6 m. Chan doan 13/10: ca 5 va cham dong cua SW15I2 deu la
    # UAV DUNG YEN sau ~2 s lap ke hoach that bai lien tuc (128-200 lan), vat can khong doi huong.
    "SW15I2E": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.099", "start_escape:=0.6"],
    # P2 hoan chinh + chong giat khi lap ke hoach cham (chay thu Indoor 13/10: A* toi 0.8 s, lenh
    # nhay 1.2 m, nghieng 75-154 do). A: A* bo cuoc sau 0.08 s. B: lap ke hoach tu trang thai
    # sau 0.2 s, gan moc thoi gian dung luc ban giao; traj_server giu quy dao cu toi luc do.
    "SW15I2EAB": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.099", "start_escape:=0.6",
                  "astar_max_time:=0.08", "replan_latency:=0.2"],
    # === P2HC: P2 hoan chinh cho Indoor === SW15I2EAB + sua loi quan goc huong mui (yaw_mode 1;
    # nguoi dung thay UAV "lieng vong" o I3 khi chua bat).
    "P2HC": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.099", "start_escape:=0.6",
             "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=1"],
    # SW15 + huong B: phep kiem va cham 20 Hz BO QUA diem qua 1.5 s toi neu diem do
    # CHI cham vet quet (khong co vat can that trong 0.15 m) -> bot lap lai [SAFETY]
    "SW15H": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5"],
    # SW15H + sua huong mui (planYaw): Y1 = sua loi quan goc; Y2 = Y1 + giu huong khi
    # UAV gan dung yen. Truoc khi sua: P2 xoay than 560-600 do/luot (P0 124 do).
    "SW15HY1": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5", "yaw_mode:=1"],
    "SW15HY2": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5", "yaw_mode:=2"],


}


def map_args(map_name):
    """Tham so roslaunch rieng cua mot ban do Indoor."""
    m = MAPS[map_name]
    base = (["use_pcd:=false"] if m.get("pcd") is None else
            ["use_pcd:=true", "map_file:=%s/%s.pcd" % (MAPS_DIR, m["pcd"])])
    return base + [
            "map_size_x:=%.1f" % m["size"][0],
            "map_size_y:=%.1f" % m["size"][1],
            "map_size_z:=%.1f" % m["size"][2],
            "init_x:=%.2f" % m["start"][0],
            "init_y:=%.2f" % m["start"][1]] + FLIGHT_ARGS + m.get("extra", [])
    # [CD2] extra dat SAU FLIGHT_ARGS: roslaunch lay gia tri CUOI, nen ban do ghi de duoc


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
        # [CD2] moi file ket qua di kem MOT file cham tran cung ten, duoi _enc
        "enc_csv:=%s" % (os.path.splitext(out_csv)[0] + "_enc.csv"),
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
    # [CD2] vat can dong: so luong; moi luot mot hat giong = so luot + 1 nen
    # hai phuong an chay cung --trials se gap DUNG cung mot kich ban
    ap.add_argument("--dyn", type=int, default=0,
                    help="so vat can dong (0 = tat; -1 = so mac dinh theo ban do trong nha)")
    ap.add_argument("--seed0", type=int, default=1,
                    help="hat giong cua luot dau (luot t dung hat seed0 + t); 41 = bo hat 41-80")
    ap.add_argument("--extra", nargs="*", default=[],
                    help="tham số roslaunch bổ sung, ví dụ dynamic_env:=1")
    a = ap.parse_args()
    
    for m in a.maps:
        if m not in MAPS:
            sys.exit("Ban do %s chua co trong MAPS" % m)
        if MAPS[m].get("pcd") is None:
            continue                       # [CD2] ban do sinh ngau nhien
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
                if a.dyn != 0:   # [CD2] dat truoc --extra de van ghi de duoc
                    di = DYN_INDOOR.get(m)
                    n = a.dyn if a.dyn > 0 else (di["num"] if di else 8)
                    extra = ["dyn_obs:=1", "dyn_num:=%d" % n,
                             "dyn_seed:=%d" % (t + a.seed0)] + extra
                    if di:   # [CD2 Indoor]
                        extra = ["dyn_xy_x:=%.1f" % di["xy"][0], "dyn_xy_y:=%.1f" % di["xy"][1],
                                 "dyn_z_margin:=0.15", "dyn_spawn:=%s" % di["spawn"]] + extra
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
