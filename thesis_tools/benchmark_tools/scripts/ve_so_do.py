#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
ve_so_do.py — Ve so do mat bang cac ban do Indoor cho luan van.

Doc thang file .pcd rồi ve hai hinh chieu:
  - hinh chieu bang: lat cat tai cao do bay z = 1.0 m
  - hinh chieu dung: lat cat doc mot duong y = const di qua duong bay

Cach dung:
    python3 ve_so_do.py            # ve tat ca ban do co trong BANDO
    python3 ve_so_do.py I3 I4      # chi ve hai ban do nay
"""

import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

MAPS_DIR = os.path.expanduser(
    "~/fast_planner_ws/src/Fast-Planner/uav_simulator/map_generator/maps")

# cut_y: vi tri lat cat cua hinh chieu dung
# tuyen: (ten, xuat phat, dich) — trung voi MAPS trong batch_runner.py
BANDO = {
    "I1": dict(cut_y=0.0,
               tuyen=[("", (-6.0, 0.0), (6.0, 0.0))],
               ghi_chu=[]),
    "I3": dict(cut_y=-8.5,
               tuyen=[("", (-8.5, -8.5), (8.5, 8.5))],
               ghi_chu=[("hanh lang chu L", (-2.0, -8.0)),
                        ("goc cua", (5.0, -5.0))]),
    "I4": dict(cut_y=-2.5,
               tuyen=[("R1", (-10.0, -5.0), (10.0, -5.0)),
                      ("R2", (-10.0, -5.0), (-10.0, 5.0)),
                      ("R3", (10.0, -5.0), (10.0, 5.0))],
               ghi_chu=[("cua A  1.4 m", (0.4, -2.5)),
                        ("cua B  1.2 m", (0.4, 2.5)),
                        ("cua C  1.2 m", (-6.0, 0.4)),
                        ("cua D  0.9 m", (6.0, 0.4))]),
}

Z_BAY = 1.0      # cao do bay
DAY = 0.05       # nua be day lat cat hinh chieu bang
DAY_DUNG = 0.12  # lat cat hinh chieu dung day hon, de bat duoc san va tran
                 # (san ve thua, cac diem cach nhau 0.2 m)


def doc_pcd(path):
    raw = open(path, "rb").read()
    i = raw.index(b"DATA binary\n") + len("DATA binary\n")
    return np.frombuffer(raw[i:], dtype=np.float32).reshape(-1, 3).astype(float)


def ve(ten):
    cfg = BANDO[ten]
    P = doc_pcd(os.path.join(MAPS_DIR, "%s.pcd" % ten))
    bang = P[np.abs(P[:, 2] - Z_BAY) < DAY]
    dung = P[np.abs(P[:, 1] - cfg["cut_y"]) < DAY_DUNG]

    fig, (a1, a2) = plt.subplots(
        2, 1, figsize=(10, 9),
        gridspec_kw=dict(height_ratios=[max(2.0, 3.0), 1.2]))

    a1.scatter(bang[:, 0], bang[:, 1], s=0.4, c="#2c3e50", linewidths=0)
    a1.set_title("Hinh chieu bang — lat cat tai cao do bay z = %.1f m" % Z_BAY)
    a1.set_ylabel("y  (m)")
    a1.set_aspect("equal")

    gom = {}   # nhieu tuyen dung chung mot diem thi gop nhan lam mot
    for nhan, xp, dich in cfg["tuyen"]:
        a1.plot([xp[0], dich[0]], [xp[1], dich[1]], "--", c="#95a5a6", lw=1.2,
                zorder=2)
        a1.plot(xp[0], xp[1], "o", c="#27632a", ms=8, zorder=4)
        a1.plot(dich[0], dich[1], "*", c="#9c4a1a", ms=15, zorder=4)
        # nhan dat lech khoi duong noi, tranh de len tuong
        gx = xp[0] + 0.35 * (dich[0] - xp[0])
        gy = xp[1] + 0.35 * (dich[1] - xp[1])
        if nhan:
            a1.annotate(nhan, (gx, gy), fontsize=11, color="#34495e",
                        fontweight="bold", textcoords="offset points",
                        xytext=(0, 8), ha="center")
        gom.setdefault((round(xp[0], 2), round(xp[1], 2)), []).append(
            ("XP", nhan))
        gom.setdefault((round(dich[0], 2), round(dich[1], 2)), []).append(
            ("dich", nhan))
    for (px, py), ds in gom.items():
        text = ", ".join(("%s %s" % (v, n)).strip() for v, n in ds)
        mau = "#27632a" if ds[0][0] == "XP" else "#9c4a1a"
        dy = -16 if ds[0][0] == "XP" else 13
        a1.annotate(text, (px, py), fontsize=8, color=mau, ha="center",
                    textcoords="offset points", xytext=(0, dy))
    for text, (tx, ty) in cfg["ghi_chu"]:
        a1.annotate(text, (tx, ty), fontsize=9, color="#c0392b", zorder=5)

    a2.scatter(dung[:, 0], dung[:, 2], s=0.4, c="#2c3e50", linewidths=0)
    a2.axhline(Z_BAY, ls="--", c="#7f8c8d", lw=1.0)
    a2.set_title("Hinh chieu dung — lat cat y = %.1f m  (net dut: cao do bay)"
                 % cfg["cut_y"])
    a2.set_xlabel("x  (m)")
    a2.set_ylabel("z  (m)")
    a2.set_aspect("equal")

    for a in (a1, a2):
        a.grid(alpha=0.25)
        for s in a.spines.values():
            s.set_visible(False)

    out = os.path.join(MAPS_DIR, "%s_so_do.png" % ten)
    fig.tight_layout()
    fig.savefig(out, dpi=110)
    plt.close(fig)
    print("%s: %d diem -> %s" % (ten, len(P), out))


def main():
    ten_list = sys.argv[1:] or sorted(BANDO)
    for t in ten_list:
        if t not in BANDO:
            sys.exit("Chua co cau hinh ve cho ban do %s" % t)
        ve(t)


if __name__ == "__main__":
    main()
