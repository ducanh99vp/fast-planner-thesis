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
    "I3T": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),  # [CD2 19/10] vat can sat tuong trai
    "I3T2": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),  # [CD2 26/10] 2 hang: trai + giua
    "I3T2B": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),  # [CD2 26/10] 2 hang nhe: 2 trai + 2 giua
    "I3T5": dict(pcd="I3", size=(21.0, 21.0, 4.5), start=(-8.5, -8.5), goals=[(8.5, 8.5)]),  # [CD2 02/11] I3T2 thua hon: 3 trai + 2 giua
    # I4: mot ban do, ba tuyen bay qua ba o cua rong khac nhau.
    #   R1 qua cua A 1.4 m, R2 qua cua C 1.2 m, R3 qua cua D 0.9 m (hep nhat)
    "I4-R1": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(10.0, -5.0)]),
    "I4-R2": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(-10.0, 5.0)]),
    "I4-R3": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(10.0, -5.0), goals=[(10.0, 5.0)]),
    "I4D-R1": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(10.0, -5.0)]),  # [CD2 02/11] I4 day: moi phong 5 vat can
    "I4D-R2": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(-10.0, -5.0), goals=[(-10.0, 5.0)]),  # [CD2 02/11] I4 day: moi phong 5 vat can
    "I4D-R3": dict(pcd="I4", size=(26.0, 16.0, 4.5), start=(10.0, -5.0), goals=[(10.0, 5.0)]),  # [CD2 02/11] I4 day: moi phong 5 vat can
    # I5: ngo cut chu U kin toi tran, mieng quay ve xuat phat — cuc tieu dia phuong
    "I5": dict(pcd="I5", size=(21.0, 16.0, 4.5), start=(-8.0, 0.0), goals=[(8.0, 0.0)]),
    "I5D": dict(pcd="I5", size=(21.0, 16.0, 4.5), start=(-8.0, 0.0), goals=[(8.0, 0.0)]),  # [CD2 02/11] I5 6 vat can dong
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
    # [CD2 19/10] I3T: ca 4 vat can di doc hanh lang DUOI, sat tuong TRAI (phia dich, y = -7.0): tam y ~ -7.45
    # -> khe trai 0.15-0.2 m, khong lot duoc; ben phai trong ~2.2 m. Thu UAV co biet lach sang phai.
    "I3T":   dict(num=4, xy=(10.0, 10.0), spawn="-6.5,6.5,-7.5,-7.4,0"),
    # [CD2 26/10] I3T2 (nguoi dung yeu cau): them 1 HANG giua hanh lang. 4 vat can sat tuong trai (y ~ -7.45)
    # + 3 vat can giua (y ~ -8.5), deu chay doc truc x. Khe giua hai hang ~0.45 m (khong lot);
    # loi con lai: sat mep phai (~1.2 m) hoac cho hang giua di qua.
    "I3T2":  dict(num=7, xy=(10.0, 10.0), spawn="-6.5,6.5,-7.5,-7.4,0,4;-6.5,6.5,-8.6,-8.4,0,3"),
    # [CD2 26/10] I3T2B (nguoi dung): ban nhe cua I3T2 — 2 vat can sat tuong trai + 2 vat can giua hanh lang
    "I3T2B": dict(num=4, xy=(10.0, 10.0), spawn="-6.5,6.5,-7.5,-7.4,0,2;-6.5,6.5,-8.6,-8.4,0,2"),
    # [CD2 02/11] I3T5 (nguoi dung): I3T2 (7 vat can) qua day so voi hanh lang 3 m -> 5 vat can: 3 sat tuong
    # trai + 2 giua. Hinh dang xen ke theo thu tu (chan = hop, le = tru) -> 3 hop + 2 tru:
    # lan trai hop-tru-hop, lan giua tru-hop.
    "I3T5":  dict(num=5, xy=(10.0, 10.0), spawn="-6.5,6.5,-7.5,-7.4,0,3;-6.5,6.5,-8.6,-8.4,0,2"),
    "I4-R1": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    "I4-R2": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    "I4-R3": dict(num=6, xy=(12.5, 7.5), spawn="-8,-1,-7,-1;1,8,-7,-1;-8,-1,1,7;1,8,1,7"),
    # [CD2 02/11] I4D (nguoi dung): moi phong du 5 vat can = tinh + dong. Tay Nam 2 tinh -> 3 dong, Dong Nam
    # 3 tinh -> 2 dong, Tay Bac / Dong Bac 2 tinh (1 khoi + 1 ban thap 0.75 m) -> 3 dong; tong 11 (I4 cu: 6).
    # Truong thu 6 = so vat can co dinh cua vung; truc -1 = quy tac chung. Hinh xen ke -> 6 hop + 5 tru.
    "I4D-R1": dict(num=11, xy=(12.5, 7.5), spawn="-8,-1,-7,-1,-1,3;1,8,-7,-1,-1,2;-8,-1,1,7,-1,3;1,8,1,7,-1,3"),
    "I4D-R2": dict(num=11, xy=(12.5, 7.5), spawn="-8,-1,-7,-1,-1,3;1,8,-7,-1,-1,2;-8,-1,1,7,-1,3;1,8,1,7,-1,3"),
    "I4D-R3": dict(num=11, xy=(12.5, 7.5), spawn="-8,-1,-7,-1,-1,3;1,8,-7,-1,-1,2;-8,-1,1,7,-1,3;1,8,1,7,-1,3"),
    "I5":    dict(num=4, xy=(10.0, 7.5), spawn="-6,1,4.8,7;-6,1,-7,-4.8;3,6.5,-7,7"),
    # [CD2 02/11] I5D (nguoi dung): 6 vat can dong, CO DINH 2 moi vung: hanh lang tren + duoi (chay doc truc x),
    # vung sau chu U (chay doc truc y) -> khong bat qua lai giua hai vach hep. Hinh xen ke -> 3 hop + 3 tru.
    "I5D":   dict(num=6, xy=(10.0, 7.5), spawn="-6,1,4.8,7,0,2;-6,1,-7,-4.8,0,2;3,6.5,-7,7,1,2"),
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
    # [16/10] tat tran ao trong nha (virtual_ceil -1): chi dung san/tran THAT nhu CD1 (M3B/TN2);
    # truoc do tran ao 2.5 tu bat vi applyVirtualBounds chay ca nhanh dam may diem.
    "P2HC": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.099", "start_escape:=0.6",
             "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=1", "virtual_ceil:=-1.0"],
    # [CD2 16/10] P2HC + (1) FSM khong bo dich khi quy dao het truoc dich (loi goc M5)
    # + (2) vet quet phinh 0.2 = vung phinh tinh = nguong va cham (truoc 0.099: quy dao "trong" van suot
    # 0.13-0.18 m) + (3) yaw_mode 2 giu mui khi bay cham (I1 hat 20: quay mui khi cham 366 -> 9 do)
    "P2HC2": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
              "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
              "goal_retry:=true"],
    # [CD2 16/10] P2HC2 + goal_retry tra lai dich THAT (dich bi vet quet de chi doi TAM) + ne ngang
    # khoi vet quet phia truoc vat can dang lao toi (cach mep 0.25 m)
    "P2HC3": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
              "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
              "goal_retry:=true", "dyn_dodge:=0.25"],
    # [CD2 17/10] tham so GIONG HET P2HC2 nhung chay tren ma da co tra lai dich that (goal_true_);
    # = P2HC3 bo ne ngang -> tach rieng loi ich cua tra dich
    "P2HC2T": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
               "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
               "goal_retry:=true"],
    # [CD2 21/10] TN2 + P2 + dai do cao: dan duong toan cuc CD1 (tri nho + A* toan cuc + dich tam), duong
    # toan cuc va tam nhin BO QUA vet quet (dich tam xa doc hanh lang), dich tam roi vao vet quet thi doi
    # tien doc duong; A* cuc bo chi bay z 0.5-2.0 m. Khoang ho A* toan cuc 0.1 vi phinh tinh da 0.2
    # (tong 0.3 m, bang CD1: 0.099 + 0.2). Ly do: log A* I3T hat 2 — heuristic thang toi dich sau tuong
    # lam A* khong xet nua phai hanh lang (0/2239 nut), lui va bay qua dau tru.
    "TN2P2Z": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
               "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
               "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
               "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0"],
    # [CD2 25/10] TN2P2Z + du doan biet tuong: duong du doan vat can cham vat can tinh thi phan xa
    # (I4-R3 hat 1: hop bat lai o khung cua, vet quet thang xuyen tuong -> UAV vao cua -> cham)
    "TN2P2ZR": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
                "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true"],
    # [CD2 25/10] TN2P2ZR + P5 toi thieu: lap ke hoach that bai ma quy dao cu sap cham vung bi chiem thi
    # phanh deu dung lai thay vi bay tiep (I4-R3 hat 1: 2.6 s that bai lien tuc, quy dao cu dua vao cua -> cham)
    "TN2P2ZRS": ["dyn_sweep_T:=1.5", "inflation:=0.2", "dyn_sweep_inflate:=0.2", "start_escape:=0.6",
                 "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                 "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                 "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                 "stop_on_fail:=true"],
    # [CD2 P3] TN2 + P3 (vat can dong THEO THOI GIAN: A* loai nut cach vat can du doan tai t < 0.3 m, f_d
    # trong bo toi uu, phep kiem 20 Hz theo thoi gian) + du doan biet tuong + dai do cao. TAT vet quet
    # (dyn_sweep_T -1): ban do chi con vat can tinh. Giu tra dich, chong giat, giu mui.
    "TN2P3": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
              "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
              "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
              "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
              "p3:=true"],
    # [CD2 P3L] TN2P3 + duong toan cuc co chi phi MEM theo du doan vat can dong (lan sap dong dat x5) -> dich tam
    # chon lan trong (I3T2 hat 2: dich tam TN2 nam lan trai dong -> UAV bi kep giua 2 lan -> cham)
    "TN2P3L": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
               "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
               "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
               "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
               "p3:=true", "guide_dyn_soft:=4.0"],
    # [CD2 P3 26/10] TN2P3L + du doan hai vat can dong DUNG NHAU (I3T2B hat 2: hop lan giua dung tru di nguoc
    # chieu, bat nguoc vao UAV dang bam sau lung no; du doan cu chi biet bat o vat can tinh)
    "TN2P3LD": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
                "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                "p3:=true", "guide_dyn_soft:=4.0", "dyn_pair:=true"],
    # [CD2 P3 26/10] TN2P3LD + bien phep kiem 20 Hz 0.3 m (0.2 = dung bang nguong sach -> I3T2B hat 2 chui khe
    # 0.45 m giua tru lan trai va hop lan giua, suot 0.19)
    "TN2P3LDM": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
                 "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                 "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                 "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                 "p3:=true", "guide_dyn_soft:=4.0", "dyn_pair:=true", "p3_check_margin:=0.3"],
    # [CD2 P3 26/10] TN2P3LDM + bong bong thoat khong cho lai gan vat can hon min(luc xuat phat, 0.2 m)
    # (I4-R2 hat 7: xuat phat cach tru 0.29 m, duong ban toi dich cat mui tru 0.11 m nam tron trong bong bong)
    "TN2P3LDMB": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
                  "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                  "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                  "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                  "p3:=true", "guide_dyn_soft:=4.0", "dyn_pair:=true", "p3_check_margin:=0.3",
                  "p3_escape_floor:=0.2"],
    # [CD2 P3 27/10] TN2P3LDM + doi dich TAM theo du doan P3 khi vat can dong sap quet qua dich (KHONG kem
    # bong bong chat cua TN2P3LDMB: no doi va cham lay nghieng 45 do). I4-R2 hat 7: tru di ngang dich.
    "TN2P3LDMG": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
                  "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                  "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                  "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                  "p3:=true", "guide_dyn_soft:=4.0", "dyn_pair:=true", "p3_check_margin:=0.3",
                  "goal_dyn:=3.0"],
    # [CD2 04/11] TN2P3LDMG + bo toi uu nhin xa 3 s (bang A*, mac dinh 1.5). I3 hat 7: A* va phep kiem thay
    # vat can tu 2.2 s truoc, f_d chi thay 1.5 s -> phan ung tre, het kip vong sang ben rong.
    "TN2P3LDMGT3": ["dyn_sweep_T:=-1.0", "inflation:=0.2", "start_escape:=0.6",
                    "astar_max_time:=0.08", "replan_latency:=0.2", "yaw_mode:=2", "virtual_ceil:=-1.0",
                    "goal_retry:=true", "keep_memory:=true", "global_guide:=true", "guide_ignore_dyn:=true",
                    "guide_clearance:=0.1", "search_zmin:=0.5", "search_zmax:=2.0", "dyn_reflect:=true",
                    "p3:=true", "guide_dyn_soft:=4.0", "dyn_pair:=true", "p3_check_margin:=0.3",
                    "goal_dyn:=3.0", "p3_tdyn:=3.0"],
    # SW15 + huong B: phep kiem va cham 20 Hz BO QUA diem qua 1.5 s toi neu diem do
    # CHI cham vet quet (khong co vat can that trong 0.15 m) -> bot lap lai [SAFETY]
    "SW15H": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5"],
    # SW15H + sua huong mui (planYaw): Y1 = sua loi quan goc; Y2 = Y1 + giu huong khi
    # UAV gan dung yen. Truoc khi sua: P2 xoay than 560-600 do/luot (P0 124 do).
    "SW15HY1": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5", "yaw_mode:=1"],
    "SW15HY2": ["dyn_sweep_T:=1.5", "dyn_check_horizon:=1.5", "yaw_mode:=2"],
    # === [CD1] Ban do co tri nho + dan duong toan cuc (14/10/2026) ===
    # M3B: tai tao baseline M3 tren nhanh hien tai (tran ao tat nhu M3, khi do no chua co hieu luc).
    "M3B": ["virtual_ceil:=-1.0"],
    # TN: M3B + ban do co tri nho + dan duong toan cuc (A* hinh hoc tren ban do da nho, dai z
    # 0.5-2.0 m, dich tam = diem xa nhat nhin thay duoc trong 6 m theo duong).
    "TN": ["virtual_ceil:=-1.0", "keep_memory:=true", "global_guide:=true"],
    "TN2": ["virtual_ceil:=-1.0", "keep_memory:=true", "global_guide:=true"],   # = TN, chi doi nhan: do bang ma 15/10 (lap lai truoc khi het quy dao + dich tam khuat gan); TN trong cd1_tn_20261014.csv la ma 14/10


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
