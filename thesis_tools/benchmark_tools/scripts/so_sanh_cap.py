#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""[CD2] So CAP hai phuong an tren cung bo hat giong.

    python3 so_sanh_cap.py A.csv B.csv [--thay B_sua.csv:TRIAL ...]

A, B do batch_runner sinh ra voi CUNG --trials, nen luot thu k cua hai file
gap cung mot kich ban vat can (hat = k + 1). --thay dung de thay mot dong hong
(vi du may ngu giua luot) bang mot lan chay lai rieng: file chay lai co 1 dong,
TRIAL la so luot (bat dau tu 0) can thay.

Quy tac ket luan (giu tu M5/M6): mot khac biet chi duoc coi la that khi CA
kiem dinh so cap (Wilcoxon / dau / McNemar) LAN kiem dinh khong so cap
(Mann-Whitney / Fisher) deu cho p < 0.05.
"""
import csv
import math
import os
import sys

from scipy import stats

L_REF = 38.0
UAV_RADIUS = 0.2


def doc(path):
    with open(path, newline="") as f:
        return list(csv.DictReader(f))


def nap(path, thay):
    """Tra ve dict trial -> dong, va dict trial -> danh sach lan cham tran."""
    luot = {r["trial"]: r for r in doc(path)}
    enc = {}
    pe = os.path.splitext(path)[0] + "_enc.csv"
    if os.path.isfile(pe):
        for e in doc(pe):
            enc.setdefault(e["trial"], []).append(e)
    for fp, t in thay:
        r = doc(fp)[0]
        r = dict(r, trial=t)
        luot[t] = r
        pe2 = os.path.splitext(fp)[0] + "_enc.csv"
        enc[t] = [dict(e, trial=t) for e in doc(pe2)] if os.path.isfile(pe2) else []
    return luot, enc


def f(x):
    try:
        return float(x)
    except (TypeError, ValueError):
        return float("nan")


def p_str(p):
    return "%.4f" % p if p >= 1e-4 else "%.1e" % p


def main():
    args = sys.argv[1:]
    thay_b = []
    if "--thay" in args:
        k = args.index("--thay")
        for s in args[k + 1:]:
            fp, t = s.rsplit(":", 1)
            thay_b.append((fp, t))
        args = args[:k]
    pa, pb = args[0], args[1]
    A, EA = nap(pa, [])
    B, EB = nap(pb, thay_b)
    ten_a = os.path.splitext(os.path.basename(pa))[0]
    ten_b = os.path.splitext(os.path.basename(pb))[0]
    T = sorted(set(A) & set(B), key=int)
    n = len(T)

    print("=" * 76)
    print("  A = %s   |   B = %s   |   %d hat chung" % (ten_a, ten_b, n))
    print("=" * 76)

    # ---------- theo LUOT ----------
    ca = [int(f(A[t]["n_hit"]) == 0) for t in T]
    cb = [int(f(B[t]["n_hit"]) == 0) for t in T]
    b_hon = sum(1 for x, y in zip(ca, cb) if y and not x)
    a_hon = sum(1 for x, y in zip(ca, cb) if x and not y)
    p_dau = stats.binomtest(b_hon, b_hon + a_hon, 0.5).pvalue if b_hon + a_hon else 1.0
    p_fis = stats.fisher_exact([[sum(cb), n - sum(cb)], [sum(ca), n - sum(ca)]])[1]
    print("  THEO LUOT — khong va cham dong")
    print("    A %d/%d = %.1f%%   B %d/%d = %.1f%%" % (sum(ca), n, 100.0 * sum(ca) / n,
                                                  sum(cb), n, 100.0 * sum(cb) / n))
    print("    cap lech: B sach ma A khong = %d, A sach ma B khong = %d" % (b_hon, a_hon))
    print("    kiem dau (so cap) p = %s   |   Fisher (khong so cap) p = %s" % (p_str(p_dau), p_str(p_fis)))

    ha = [f(A[t]["n_hit"]) for t in T]
    hb = [f(B[t]["n_hit"]) for t in T]
    p_w = stats.wilcoxon(ha, hb).pvalue if any(x != y for x, y in zip(ha, hb)) else 1.0
    p_m = stats.mannwhitneyu(ha, hb).pvalue
    print("  SO VA CHAM DONG moi luot: A tong %d, B tong %d" % (sum(ha), sum(hb)))
    print("    Wilcoxon (so cap) p = %s   |   Mann-Whitney p = %s" % (p_str(p_w), p_str(p_m)))

    # ---------- theo LAN CHAM TRAN ----------
    def gom(E):
        d = [f(e["d_min"]) for t in T for e in E.get(t, [])]
        h = [int(f(e["hit"]) > 0) for t in T for e in E.get(t, [])]
        return d, h
    da, xa = gom(EA)
    db, xb = gom(EB)
    pf = stats.fisher_exact([[len(xb) - sum(xb), sum(xb)], [len(xa) - sum(xa), sum(xa)]])[1]
    print("  THEO LAN CHAM TRAN")
    print("    A ne %d/%d = %.1f%%   B ne %d/%d = %.1f%%   (Fisher p = %s)" % (
        len(xa) - sum(xa), len(xa), 100.0 * (len(xa) - sum(xa)) / len(xa),
        len(xb) - sum(xb), len(xb), 100.0 * (len(xb) - sum(xb)) / len(xb), p_str(pf)))
    print("    LUU Y: cac lan trong cung mot luot khong doc lap, nen p o tren lac quan;")
    print("    so sanh dang tin la khoang ho NHO NHAT moi luot (duoi day).")

    ma = [f(A[t]["d_min_dyn"]) for t in T]
    mb = [f(B[t]["d_min_dyn"]) for t in T]
    med = lambda v: sorted(v)[len(v) // 2]
    print("  KHOANG HO NHO NHAT MOI LUOT (m): A trung vi %.3f   B trung vi %.3f" % (med(ma), med(mb)))
    print("    Wilcoxon p = %s   |   Mann-Whitney p = %s   |   B tot hon o %d/%d hat" % (
        p_str(stats.wilcoxon(ma, mb).pvalue), p_str(stats.mannwhitneyu(ma, mb).pvalue),
        sum(1 for x, y in zip(ma, mb) if y > x), n))

    # ---------- gia phai tra ----------
    ea = [f(A[t]["L"]) / L_REF for t in T]
    eb = [f(B[t]["L"]) / L_REF for t in T]
    print("  ETA = L / %.0f m:  A tb %.3f (tv %.3f)   B tb %.3f (tv %.3f)" % (
        L_REF, sum(ea) / n, med(ea), sum(eb) / n, med(eb)))
    print("    Wilcoxon p = %s   |   Mann-Whitney p = %s" % (
        p_str(stats.wilcoxon(ea, eb).pvalue), p_str(stats.mannwhitneyu(ea, eb).pvalue)))
    ta = [f(A[t]["T_f"]) for t in T]
    tb = [f(B[t]["T_f"]) for t in T]
    print("  T_f (s):  A tb %.1f   B tb %.1f" % (sum(ta) / n, sum(tb) / n))
    ra = [f(A[t]["N_replan"]) for t in T]
    rb = [f(B[t]["N_replan"]) for t in T]
    print("  Lap lai ke hoach:  A tb %.1f (max %d)   B tb %.1f (max %d)" % (
        sum(ra) / n, max(ra), sum(rb) / n, max(rb)))

    # ---------- an toan tinh ----------
    for nm, X in (("A", A), ("B", B)):
        vc = [int(t) + 1 for t in T if X[t]["collided"] == "1"]
        hg = [int(t) + 1 for t in T if X[t]["timed_out"] == "1"]
        zt = [int(t) + 1 for t in T if f(X[t]["z_min_flight"]) < 0.3]
        print("  %s: va cham TINH o hat %s | het gio o hat %s | ha xuong < 0.3 m o hat %s" % (
            nm, vc or "-", hg or "-", zt or "-"))


if __name__ == "__main__":
    main()
