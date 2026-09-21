#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
analyze.py — Mốc M2, luận văn UAV tự hành vòng tránh vật cản

Đọc CSV thô do metric_logger sinh ra, tổng hợp thành bảng kết quả
và biểu đồ dùng được ngay trong Chương 5 của luận văn.

Sinh ra:
  results/summary/bang_ket_qua.md     — bảng Markdown, đọc nhanh
  results/summary/bang_ket_qua.tex    — bảng LaTeX, dán vào luận văn
  results/summary/bang_ket_qua.csv    — bảng đã tổng hợp
  results/summary/boxplot_<chi_so>.png
  results/summary/ty_le_thanh_cong.png

Cách dùng:
    python3 analyze.py
    python3 analyze.py --csv duong/dan/khac.csv --only-success
"""

import os
import argparse

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

DEFAULT_CSV = os.path.expanduser(
    "~/fast_planner_ws/src/Fast-Planner/results/raw/bench.csv")
DEFAULT_OUT = os.path.expanduser(
    "~/fast_planner_ws/src/Fast-Planner/results/summary")

# Chỉ số nào lấy trung bình ± độ lệch chuẩn, kèm nhãn hiển thị
METRICS = [
    ("T_f",       "Thời gian bay (s)"),
    ("L",         "Độ dài quỹ đạo (m)"),
    ("v_mean",         "Vận tốc TB (m/s)"),
    ("v_cmd_axis_max", "Vận tốc lệnh cực đại theo trục (m/s)"),
    ("S_J",       "Độ mượt S_J (đã loại mối nối)"),
    ("j_rms",     "Jerk hiệu dụng (m/s³)"),
    ("dacc_max",  "Gián đoạn mối nối lớn nhất (m/s²)"),
    ("d_min",     "Khoảng cách an toàn min (m)"),
    ("d_p5",      "Khoảng cách phân vị 5% (m)"),          # [Luan van - M3]
    ("t_fe_mean", "Thời gian front-end (ms)"),
    ("t_fe_max",  "Front-end cực đại (ms)"),              # [Luan van - M3]
    ("t_be_mean", "Thời gian back-end (ms)"),
    ("t_be_p95",  "Back-end phân vị 95 (ms)"),
    ("N_replan",  "Số lần lập lại"),
    ("z_min_flight", "Độ cao bay thấp nhất (m)"),         # [Luan van - M3]
]
# [Luan van - M5] Chỉ số vật cản động — chỉ có trong CSV của đợt chạy bật dyn_obs
METRICS_DYN = [
    ("d_min_dyn",  "Khoảng cách tới vật cản động min (m)"),
    ("d_p5_dyn",   "Vật cản động, phân vị 5% (m)"),
    ("n_coll_dyn", "Số lần va chạm vật cản động"),
]
# [Luan van - M3] Lượt hết giờ bị cắt ở ngưỡng timeout, nên T_f và L của nó chỉ
# phản ánh ngưỡng chứ không phải quỹ đạo tới đích. Nhóm nào không có lượt nào
# thành công thì hai chỉ số này để "—".
CAPPED = ("T_f", "L")

# Bảng màu trung tính, hợp cả in đen trắng
COLORS = ["#3D6FA8", "#0C6E76", "#A5682B", "#6B5B95", "#4F7942"]


def load(path):
    if not os.path.isfile(path):
        raise SystemExit("Không tìm thấy file: %s" % path)
    df = pd.read_csv(path)
    if df.empty:
        raise SystemExit("File CSV rỗng: %s" % path)
    # [Luan van - M3] CSV cũ (lược đồ khác) thiếu cột thì dừng, không tổng hợp âm thầm
    need = ["map", "config", "success", "collided", "timed_out"] + [k for k, _ in METRICS]
    missing = [k for k in need if k not in df.columns]
    if missing:
        raise SystemExit("CSV %s thiếu cột: %s — file sinh từ lược đồ cũ, hãy chạy lại"
                         % (path, ", ".join(missing)))
    # [Luan van - M5] CSV co cot vat can dong thi tong hop luon
    if all(k in df.columns for k, _ in METRICS_DYN) and METRICS_DYN[0] not in METRICS:
        METRICS.extend(METRICS_DYN)
    df["d_min"] = pd.to_numeric(df["d_min"], errors="coerce")
    return df

def summarize(df, only_success):
    """Trả về DataFrame tổng hợp theo (map, config)."""
    rows = []
    for (m, c), g in df.groupby(["map", "config"], sort=True):
        n = len(g)
        sr = 100.0 * g["success"].mean()
        gs = g[g["success"] == 1] if only_success else g
        if len(gs) == 0:
            gs = g
        rec = {"map": m, "config": c, "n": n,
               "SR (%)": "%.1f" % sr,
               "n_va_cham": int(g["collided"].sum()),
               "n_qua_gio": int(g["timed_out"].sum())}
        khong_thanh_cong = int(g["success"].sum()) == 0
        for key, _ in METRICS:
            if khong_thanh_cong and key in CAPPED:
                rec[key] = "—"
                continue
            v = pd.to_numeric(gs[key], errors="coerce").dropna()
            rec[key] = "—" if len(v) == 0 else "%.2f ± %.2f" % (v.mean(), v.std(ddof=1) if len(v) > 1 else 0.0)
        rows.append(rec)
    cols = ["map", "config", "n", "SR (%)", "n_va_cham", "n_qua_gio"] + [k for k, _ in METRICS]
    return pd.DataFrame(rows)[cols]


def write_markdown(summary, path):
    labels = {k: v for k, v in METRICS}
    head = ["Bản đồ", "Cấu hình", "N", "SR (%)", "Va chạm", "Quá giờ"] + \
           [labels[k] for k, _ in METRICS]
    lines = ["# Bảng kết quả tổng hợp", "",
             "Mỗi ô: trung bình ± độ lệch chuẩn. Cột N là số lượt chạy.",
             "Dấu — ở Thời gian bay và Độ dài quỹ đạo: nhóm không có lượt nào "
             "thành công, hai chỉ số đó bị chặn bởi ngưỡng hết giờ nên vô nghĩa.", "",
             "| " + " | ".join(head) + " |",
             "|" + "|".join(["---"] * len(head)) + "|"]
    for _, r in summary.iterrows():
        cells = [str(r["map"]), str(r["config"]), str(r["n"]), str(r["SR (%)"]),
                 str(r["n_va_cham"]), str(r["n_qua_gio"])] + \
                [str(r[k]) for k, _ in METRICS]
        lines.append("| " + " | ".join(cells) + " |")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines) + "\n")


def write_latex(summary, path):
    labels = {k: v for k, v in METRICS}
    # [Luan van - M3] bảng in trong luận văn: SR + 5 chỉ số chính; j_rms thay S_J vì S_J tỷ lệ với T_f
    keys = ["T_f", "L", "j_rms", "d_min", "t_be_mean"]
    head = ["Bản đồ", "Cấu hình", "SR (\\%)"] + [labels[k].replace("_", "\\_") for k in keys]
    out = ["\\begin{table}[htbp]", "\\centering",
           "\\caption{Kết quả thực nghiệm, trung bình $\\pm$ độ lệch chuẩn trên %d lượt chạy}"
           % int(summary["n"].max()),
           "\\label{tab:ket-qua}",
           "\\begin{tabular}{ll" + "r" * (len(head) - 2) + "}", "\\hline",
           " & ".join(head) + " \\\\", "\\hline"]
    for _, r in summary.iterrows():
        cells = [str(r["map"]), str(r["config"]), str(r["SR (%)"])] + \
                [str(r[k]).replace("±", "$\\pm$") for k in keys]
        out.append(" & ".join(cells) + " \\\\")
    out += ["\\hline", "\\end{tabular}", "\\end{table}"]
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(out) + "\n")


def plot_box(df, key, label, outdir):
    configs = sorted(df["config"].unique())
    maps = sorted(df["map"].unique())
    fig, ax = plt.subplots(figsize=(1.6 * len(maps) * len(configs) + 2.5, 4.0))

    data, positions, colors, ticks, tickpos = [], [], [], [], []
    pos = 0.0
    for mi, m in enumerate(maps):
        for ci, c in enumerate(configs):
            v = pd.to_numeric(df[(df["map"] == m) & (df["config"] == c)][key],
                              errors="coerce").dropna()
            if len(v) == 0:
                continue
            data.append(v.values)
            positions.append(pos)
            colors.append(COLORS[ci % len(COLORS)])
            pos += 1.0
        tickpos.append(pos - len(configs) / 2.0 - 0.0)
        ticks.append(m)
        pos += 0.8

    if not data:
        plt.close(fig)
        return
    bp = ax.boxplot(data, positions=positions, widths=0.65, patch_artist=True,
                    medianprops=dict(color="#151D28", linewidth=1.4),
                    flierprops=dict(marker="o", markersize=3, alpha=0.5))
    for patch, col in zip(bp["boxes"], colors):
        patch.set_facecolor(col)
        patch.set_alpha(0.45)
        patch.set_edgecolor(col)
        patch.set_linewidth(1.3)

    ax.set_xticks(tickpos)
    ax.set_xticklabels(ticks)
    ax.set_ylabel(label)
    ax.set_title(label, fontsize=11, loc="left")
    ax.grid(axis="y", alpha=0.25, linewidth=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    handles = [plt.Rectangle((0, 0), 1, 1, facecolor=COLORS[i % len(COLORS)],
                             alpha=0.45, edgecolor=COLORS[i % len(COLORS)])
               for i in range(len(configs))]
    ax.legend(handles, configs, frameon=False, fontsize=9, ncol=len(configs))
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "boxplot_%s.png" % key), dpi=160)
    plt.close(fig)


def plot_success(df, outdir):
    maps = sorted(df["map"].unique())
    configs = sorted(df["config"].unique())
    w = 0.8 / max(1, len(configs))
    fig, ax = plt.subplots(figsize=(1.3 * len(maps) + 3.0, 3.8))
    x = np.arange(len(maps))
    for ci, c in enumerate(configs):
        vals = [100.0 * df[(df["map"] == m) & (df["config"] == c)]["success"].mean()
                if len(df[(df["map"] == m) & (df["config"] == c)]) else 0.0
                for m in maps]
        ax.bar(x + ci * w - 0.4 + w / 2, vals, w * 0.9,
               label=c, color=COLORS[ci % len(COLORS)], alpha=0.85)
    ax.set_xticks(x)
    ax.set_xticklabels(maps)
    ax.set_ylabel("Tỷ lệ thành công (%)")
    ax.set_ylim(0, 105)
    ax.grid(axis="y", alpha=0.25, linewidth=0.7)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.legend(frameon=False, fontsize=9, ncol=len(configs))
    fig.tight_layout()
    fig.savefig(os.path.join(outdir, "ty_le_thanh_cong.png"), dpi=160)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", default=DEFAULT_CSV)
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--only-success", action="store_true",
                    help="chỉ tính trung bình trên các lượt thành công")
    a = ap.parse_args()

    os.makedirs(a.out, exist_ok=True)
    df = load(a.csv)

    print("Đã đọc %d lượt chạy, %d bản đồ, %d cấu hình."
          % (len(df), df["map"].nunique(), df["config"].nunique()))
    n_min = df.groupby(["map", "config"]).size().min()
    if n_min < 10:
        print("CẢNH BÁO: có tổ hợp chỉ %d lượt. Cần tối thiểu 20 lượt "
              "để trung bình ± độ lệch chuẩn có ý nghĩa thống kê." % n_min)

    summary = summarize(df, a.only_success)
    summary.to_csv(os.path.join(a.out, "bang_ket_qua.csv"), index=False)
    write_markdown(summary, os.path.join(a.out, "bang_ket_qua.md"))
    write_latex(summary, os.path.join(a.out, "bang_ket_qua.tex"))

    for key, label in METRICS:
        plot_box(df, key, label, a.out)
    plot_success(df, a.out)

    print()
    print(summary.to_string(index=False))
    print()
    print("Đã ghi kết quả vào: %s" % a.out)


if __name__ == "__main__":
    main()
