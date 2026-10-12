#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[CD2] Phan tich ket qua ne vat can dong theo LAN CHAM TRAN.

Doc cap file do batch_runner sinh ra:
    <ten>.csv       mot dong moi LUOT bay
    <ten>_enc.csv   mot dong moi LAN CHAM TRAN

Vi sao khong chi dem "% luot khong va cham": luot nao UAV khong gap vat can
cung duoc tinh la thanh cong, du khong kiem tra duoc gi. Dem theo lan cham
tran va do KHOANG HO dat duoc thi do dung nang luc ne.

Dung:
    python3 phan_tich_cham_tran.py ../../../results/raw/cd2_P0_thu.csv
    python3 phan_tich_cham_tran.py a.csv b.csv      # so hai phuong an
"""
import csv
import math
import os
import sys

UAV_RADIUS = 0.2          # trung metric_logger: d < 0.2 m la va cham
L_REF = 38.0              # duong thang (-19,0) -> (19,0) cua ban do OUT

# Phan loai hinh hoc theo goc giua van toc UAV va van toc vat can
NHOM_GOC = [
    ("doi dau  (135-180)", 135.0, 180.1),
    ("cat ngang (45-135)", 45.0, 135.0),
    ("cung chieu (0-45) ", -0.1, 45.0),
]


def wilson(k, n, z=1.96):
    """Khoang tin cay Wilson 95% cho ty le k/n."""
    if n == 0:
        return (float("nan"), float("nan"))
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return (max(0.0, c - h), min(1.0, c + h))


def doc(path):
    if not os.path.isfile(path):
        return []
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def so(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def phan_vi(v, q):
    v = sorted(x for x in v if x == x)
    if not v:
        return float("nan")
    k = (len(v) - 1) * q / 100.0
    a, b = int(math.floor(k)), int(math.ceil(k))
    return v[a] + (v[b] - v[a]) * (k - a)


def bao_cao(path):
    ten = os.path.splitext(os.path.basename(path))[0]
    luot = doc(path)
    ct = doc(os.path.splitext(path)[0] + "_enc.csv")
    print("=" * 72)
    print("  %s" % ten)
    print("=" * 72)
    if not luot:
        print("  khong co du lieu luot bay")
        return None

    # ---------------- theo LUOT ----------------
    n = len(luot)
    sach = sum(1 for r in luot if so(r.get("n_hit")) == 0)
    lo, hi = wilson(sach, n)
    L = [so(r["L"]) for r in luot]
    T = [so(r["T_f"]) for r in luot]
    toi = sum(1 for r in luot if r.get("success") == "1")
    print("  THEO LUOT  (n = %d)" % n)
    print("    khong va cham dong : %d/%d = %.1f%%   [Wilson 95%%: %.1f-%.1f%%]"
          % (sach, n, 100.0 * sach / n, 100 * lo, 100 * hi))
    print("    toi dich           : %d/%d" % (toi, n))
    print("    eta = L / %.0f m    : tb %.3f   trung vi %.3f   max %.3f"
          % (L_REF, sum(L) / n / L_REF, phan_vi(L, 50) / L_REF, max(L) / L_REF))
    print("    T_f (s)            : tb %.1f   trung vi %.1f" % (sum(T) / n, phan_vi(T, 50)))
    print()

    # ---------------- theo LAN CHAM TRAN ----------------
    if not ct:
        print("  khong co file cham tran")
        return None
    d = [so(e["d_min"]) for e in ct]
    hit = [int(so(e["hit"]) > 0) for e in ct]
    m = len(ct)
    kh = sum(hit)
    lo, hi = wilson(m - kh, m)
    print("  THEO LAN CHAM TRAN  (m = %d, trung binh %.1f lan moi luot)" % (m, m / n))
    print("    ne duoc (d >= %.1f m)    : %d/%d = %.1f%%   [Wilson 95%%: %.1f-%.1f%%]"
          % (UAV_RADIUS, m - kh, m, 100.0 * (m - kh) / m, 100 * lo, 100 * hi))
    print("    khoang ho d_min (m)      : p5 %.2f   p25 %.2f   trung vi %.2f   p75 %.2f"
          % (phan_vi(d, 5), phan_vi(d, 25), phan_vi(d, 50), phan_vi(d, 75)))
    for nguong in (0.2, 0.5, 1.0):
        k = sum(1 for x in d if x < nguong)
        print("    so lan d < %.1f m         : %3d  (%.1f%%)" % (nguong, k, 100.0 * k / m))
    print()

    print("  THEO HINH HOC  (goc giua van toc UAV va vat can)")
    print("    %-20s %5s %8s %10s %12s" % ("nhom", "lan", "va cham", "% ne duoc", "d_min tv (m)"))
    for nhan, a, b in NHOM_GOC:
        sel = [(so(e["d_min"]), int(so(e["hit"]) > 0)) for e in ct
               if a <= so(e["angle_deg"]) < b]
        if not sel:
            print("    %-20s %5d" % (nhan, 0))
            continue
        k = sum(h for _, h in sel)
        print("    %-20s %5d %8d %9.1f%% %12.2f"
              % (nhan, len(sel), k, 100.0 * (len(sel) - k) / len(sel),
                 phan_vi([x for x, _ in sel], 50)))
    khong_goc = sum(1 for e in ct if so(e["angle_deg"]) != so(e["angle_deg"]))
    if khong_goc:
        print("    (khong tinh duoc goc: %d lan — UAV hoac vat can dung yen)" % khong_goc)
    print()
    return dict(n=n, sach=sach, m=m, hit=kh, d=d)


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(__doc__)
        sys.exit(1)
    for p in sys.argv[1:]:
        bao_cao(p)
