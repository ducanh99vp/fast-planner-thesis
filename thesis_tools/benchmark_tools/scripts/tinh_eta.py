#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
tinh_eta.py — Cong 5 cua M6: ty so eta = L / L_A*

L    : chieu dai quy dao THAT cua UAV, doc tu CSV benchmark
L_A* : chieu dai duong di NGAN NHAT khong va cham tren cung ban do tinh,
       tim bang A* hinh hoc 3 chieu tren luoi voxel 0.1 m

eta noi len cai gia phai tra so voi toi uu hinh hoc. Duong A* la GIOI HAN DUOI:
no khong kha thi dong hoc (goc nhon, gia toc vo han tai goc) va khong xet vat
can dong. Vi vay eta gop CA hai phan: vong tranh vat can tinh va vat can dong.

Cach dung:
    python3 tinh_eta.py
"""

import os, sys, csv, heapq, math
import numpy as np

RES      = 0.1     # bang sdf_map/resolution
INFLATE  = 1       # 1 voxel = 0.1 m, bang sdf_map/obstacles_inflation (0.099)
Z_LO, Z_HI = 0.2, 2.8    # khong chui xuong san, khong vuot tran ao

R = os.path.expanduser("~/fast_planner_ws/src/Fast-Planner")
PCD = os.path.join(R, "uav_simulator/map_generator/maps/I1.pcd")
RAW = os.path.join(R, "results/raw")
START, GOAL = (-6.0, 0.0, 0.9), (6.0, 0.0, 1.0)
CAU_HINH = [("C0 khong ne", "final_C0.csv"), ("C1 chi f_d", "final_C1.csv"),
            ("C2 f_d+20Hz", "final_C2.csv"), ("C3 bien co dinh", "final_C3.csv")]


def doc_pcd(path):
    """PCD v0.7, FIELDS x y z, TYPE F, DATA binary."""
    with open(path, "rb") as f:
        raw = f.read()
    k = raw.find(b"DATA binary\n")
    if k < 0:
        sys.exit("Chi ho tro PCD 'DATA binary': " + path)
    head = raw[:k].decode("ascii", "replace")
    n = int([l.split()[1] for l in head.splitlines() if l.startswith("POINTS")][0])
    body = raw[k + len(b"DATA binary\n"):]
    return np.frombuffer(body[:n * 12], dtype=np.float32).reshape(n, 3)


def key(i, j, k):
    return (i << 20) | (j << 10) | k          # j, k < 1024


def dung_luoi(pts):
    """Tra ve (tap voxel bi chiem da phinh, goc toa do luoi)."""
    lo = np.array([pts[:, 0].min(), pts[:, 1].min(), 0.0]) - 1.0
    idx = np.floor((pts - lo) / RES).astype(np.int64)
    occ = set()
    for i, j, k in idx:
        if 0 <= j < 1024 and 0 <= k < 1024:
            occ.add(key(int(i), int(j), int(k)))
    # phinh vat can
    inf = set(occ)
    for d in range(1, INFLATE + 1):
        them = set()
        for v in inf:
            i, j, k = v >> 20, (v >> 10) & 1023, v & 1023
            for di in (-1, 0, 1):
                for dj in (-1, 0, 1):
                    for dk in (-1, 0, 1):
                        jj, kk = j + dj, k + dk
                        if 0 <= jj < 1024 and 0 <= kk < 1024:
                            them.add(key(i + di, jj, kk))
        inf |= them
    return inf, lo


def astar(occ, lo, start, goal):
    def toi(p):
        i, j, k = np.floor((np.array(p) - lo) / RES).astype(int)
        return (int(i), int(j), int(k))
    s, g = toi(start), toi(goal)
    k_lo, k_hi = int((Z_LO - lo[2]) / RES), int((Z_HI - lo[2]) / RES)
    buoc = []
    for di in (-1, 0, 1):
        for dj in (-1, 0, 1):
            for dk in (-1, 0, 1):
                if di or dj or dk:
                    buoc.append((di, dj, dk, math.sqrt(di*di + dj*dj + dk*dk) * RES))

    def h(n):
        return math.sqrt(sum(((a - b) * RES) ** 2 for a, b in zip(n, g)))

    mo = [(h(s), 0.0, s)]
    gsc = {s: 0.0}
    cha = {}
    dong = set()
    while mo:
        f, gc, n = heapq.heappop(mo)
        if n in dong:
            continue
        dong.add(n)
        if n == g:
            duong = [n]
            while duong[-1] in cha:
                duong.append(cha[duong[-1]])
            return gc, duong[::-1]
        for di, dj, dk, c in buoc:
            m = (n[0] + di, n[1] + dj, n[2] + dk)
            if m in dong or not (k_lo <= m[2] <= k_hi):
                continue
            if key(m[0], m[1], m[2]) in occ:
                continue
            ng = gc + c
            if ng < gsc.get(m, 1e18):
                gsc[m] = ng
                cha[m] = n
                heapq.heappush(mo, (ng + h(m), ng, m))
    return None, None


def phan_tich_goc(duong):
    """Duong A* la da tuyen: dem cac goc gap — bang chung no khong kha thi."""
    if not duong or len(duong) < 3:
        return 0, 0.0
    v = np.diff(np.array(duong, dtype=float), axis=0)
    n = np.linalg.norm(v, axis=1, keepdims=True)
    v = v / np.where(n == 0, 1, n)
    cos = np.clip(np.sum(v[:-1] * v[1:], axis=1), -1.0, 1.0)
    goc = np.degrees(np.arccos(cos))
    return int((goc > 30).sum()), float(goc.max())


def main():
    print("Doc ban do: %s" % PCD)
    pts = doc_pcd(PCD)
    print("  %d diem | z tu %.2f den %.2f m" % (len(pts), pts[:, 2].min(), pts[:, 2].max()))
    thang = math.dist(START, GOAL)

    # Kiem do nhay: L_A* co phu thuoc muc phinh khong. Neu khong thi moc so sanh
    # vung, vi ket qua khong doi du doi hoi khoang cach an toan bao nhieu.
    global INFLATE
    ket = {}
    for inf in (1, 2, 4):
        INFLATE = inf
        occ, lo = dung_luoi(pts)
        L, duong = astar(occ, lo, START, GOAL)
        if L is None:
            print("  phinh %.1f m: A* KHONG tim duoc duong" % (inf * RES)); continue
        z = [lo[2] + (n[2] + 0.5) * RES for n in duong]
        n_goc, goc_max = phan_tich_goc(duong)
        ket[inf] = (L, n_goc, goc_max, min(z), max(z))
        print("  phinh %.1f m (%7d voxel): L_A* = %.3f m | %d goc > 30 do | goc gap nhat %.0f do | z %.2f-%.2f m"
              % (inf * RES, len(occ), L, n_goc, goc_max, min(z), max(z)))
    if 1 not in ket:
        sys.exit("Khong tinh duoc L_A*")
    L_astar, n_goc, goc_max, z_lo, z_hi = ket[1]

    print("\nDuong ngan nhat hinh hoc dung lam moc (phinh 0.1 m):")
    print("  L_A*        = %.3f m   (duong thang %.3f m, ty le %.3f)"
          % (L_astar, thang, L_astar / thang))
    if len(ket) > 1 and max(v[0] for v in ket.values()) - min(v[0] for v in ket.values()) < 0.05:
        print("  L_A* KHONG doi khi tang muc phinh tu 0.1 len 0.4 m => hanh lang thong")
        print("  thoang it nhat 0.4 m, nen moc nay khong phu thuoc gia tri phinh.")
    print("  %d goc > 30 do, goc gap nhat %.0f do => da tuyen nay KHONG kha thi" % (n_goc, goc_max))
    print("  dong hoc: tai goc %.0f do ban kinh cong bang 0 nen gia toc phap tuyen" % goc_max)
    print("  v^2/r la vo han. Day la GIOI HAN DUOI ly thuyet, khong phai quy dao bay duoc.")

    print("\n%-17s %-8s %-9s %-14s %-16s" % ("cau hinh", "so luot", "L (m)", "eta = L/L_A*", "eta nho nhat-lon nhat"))
    for nhan, tep in CAU_HINH:
        p = os.path.join(RAW, tep)
        if not os.path.isfile(p):
            print("%-17s (thieu %s)" % (nhan, tep)); continue
        Ls = []
        for r in csv.DictReader(open(p)):
            try: Ls.append(float(r["L"]))
            except Exception: pass
        if not Ls: continue
        e = np.array(Ls) / L_astar
        print("%-17s %-8d %-9.2f %-14.3f %.3f - %.3f"
              % (nhan, len(Ls), float(np.mean(Ls)), float(e.mean()), e.min(), e.max()))

    print("\nGhi chu khi viet luan van:")
    print("  - L_A* tinh tren ban do TINH. Vat can dong khong co duong ngan nhat xac")
    print("    dinh vi chung di chuyen, nen eta gop CA chi phi vong vat can tinh VA")
    print("    chi phi ne vat can dong. Phai noi ro dieu nay.")
    print("  - eta cua C0 (khong ne vat can dong) do rieng phan thu nhat.")
    print("  - Muc phinh bang sdf_map/obstacles_inflation, tuc A* duoc huong dieu kien")
    print("    de dai nhat: moi eta > 1 deu la chi phi THAT, khong phai do thien lech.")


if __name__ == "__main__":
    main()
