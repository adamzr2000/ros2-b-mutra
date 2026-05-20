#!/usr/bin/env python3
"""
Grouped boxplot: /tf inter-message interval — SSP × CPU parameter sweep.

X-groups : SSP ∈ {5, 1, 0.1, 0.01} s
Within each group: CPU ∈ {NC, 1.0, 0.5, 0.1} — one box per CPU config.
Baseline (no sidecar) shown as a gray IQR band + dashed median.

Box  : Q1 – Q3  (IQR)
Line : median
Whiskers : Q1 − 1.5×IQR  to  Q3 + 1.5×IQR  (Tukey standard)
Fliers   : hidden
"""

import re
from pathlib import Path

import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

RESULTS_DIR = Path(__file__).parent.parent / "data/robot-benchmark/results/tf"
FONT_SCALE  = 1.7

SSP_MS_LIST = [5000, 1000, 100, 10]          # 5 s, 1 s, 0.1 s, 0.01 s
CPU_CONFIGS = [("NC", "NC"), ("1.0", 1.0), ("0.5", 0.5), ("0.1", 0.1)]
# (legend label, filter value)  — NC means uncapped

BOX_WIDTH  = 0.18   # width of each individual box
GROUP_GAP  = 0.35   # extra space between SSP groups

_TAGGED_RE = re.compile(
    r"SSP(?P<ssp>\d+)ms-ITERQ\d+-cpu(?P<cpu>[\dp]+|NC)-run\d+\.csv$",
    re.IGNORECASE,
)

COLOR_BASELINE = "#888888"
# One color per CPU config (tab10 subset — readable on white)
_t10 = sns.color_palette("tab10")
CPU_COLORS = {
    "NC":  _t10[0],   # blue
    "1.0": _t10[1],   # orange
    "0.5": _t10[2],   # green
    "0.1": _t10[3],   # red
}


def _intervals_ms(csv_path: Path) -> np.ndarray:
    df = pd.read_csv(csv_path)
    parts = []
    for _, grp in df.groupby("robot"):
        grp = grp.sort_values("wall_stamp_ns")
        iv = grp["wall_stamp_ns"].diff().dropna().values / 1e6
        parts.append(iv[iv > 0])
    return np.concatenate(parts) if parts else np.array([])


def _collect_baseline(results_tf: Path) -> np.ndarray:
    csvs = sorted((results_tf / "no_sidecar" / "baseline").rglob("*.csv"))
    parts = [_intervals_ms(p) for p in csvs]
    return np.concatenate(parts) if parts else np.array([])


def _collect_sidecar(results_tf: Path, ssp_ms: int, cpu) -> np.ndarray:
    parts = []
    for p in sorted((results_tf / "with_sidecar" / "continuous").rglob("*.csv")):
        m = _TAGGED_RE.search(p.name)
        if not m:
            continue
        if int(m.group("ssp")) != ssp_ms:
            continue
        file_cpu = m.group("cpu").upper()
        if isinstance(cpu, str) and cpu.upper() == "NC":
            if file_cpu != "NC":
                continue
        else:
            if file_cpu == "NC":
                continue
            if abs(float(file_cpu.replace("P", ".")) - float(cpu)) > 1e-6:
                continue
        parts.append(_intervals_ms(p))
    return np.concatenate(parts) if parts else np.array([])


def _print_stats(label: str, iv: np.ndarray) -> None:
    if iv.size == 0:
        print(f"[{label}]  no data")
        return
    q1, med, q3 = np.percentile(iv, [25, 50, 75])
    print(f"[{label}]  n={iv.size:,}  median={med:.2f} ms  "
          f"IQR=[{q1:.2f}, {q3:.2f}]  p95={np.percentile(iv, 95):.2f} ms")


def main():
    script_dir = Path(__file__).parent.resolve()

    if not RESULTS_DIR.exists():
        raise SystemExit(f"Results dir not found: {RESULTS_DIR}")

    iv_baseline = _collect_baseline(RESULTS_DIR)
    _print_stats("baseline", iv_baseline)

    # ── collect all data ────────────────────────────────────────────────────────
    # data[ssp_ms][cpu_label] = intervals array
    data = {}
    for ssp_ms in SSP_MS_LIST:
        data[ssp_ms] = {}
        for cpu_label, cpu_val in CPU_CONFIGS:
            iv = _collect_sidecar(RESULTS_DIR, ssp_ms=ssp_ms, cpu=cpu_val)
            data[ssp_ms][cpu_label] = iv
            _print_stats(f"SSP={ssp_ms}ms  CPU={cpu_label}", iv)

    # ── compute x positions ────────────────────────────────────────────────────
    n_cpu  = len(CPU_CONFIGS)
    # center offsets within a group: symmetric around 0
    offsets = np.linspace(-(n_cpu - 1) / 2, (n_cpu - 1) / 2, n_cpu) * (BOX_WIDTH + 0.04)

    group_centers = []
    c = 0.0
    for _ in SSP_MS_LIST:
        group_centers.append(c)
        c += 1.0 + GROUP_GAP

    # ── plot ────────────────────────────────────────────────────────────────────
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, ax = plt.subplots(figsize=(9.5, 4.2))
    plt.subplots_adjust(left=0.09, right=0.97, top=0.95, bottom=0.22)

    # baseline band
    if iv_baseline.size:
        q1, med, q3 = np.percentile(iv_baseline, [25, 50, 75])
        ax.axhspan(q1, q3, color=COLOR_BASELINE, alpha=0.18, zorder=1)
        ax.axhline(med, color=COLOR_BASELINE, linewidth=1.4, linestyle="--", zorder=2)

    all_iv_parts = [iv_baseline] if iv_baseline.size else []

    for gi, (ssp_ms, gc) in enumerate(zip(SSP_MS_LIST, group_centers)):
        for ci, ((cpu_label, _), offset) in enumerate(zip(CPU_CONFIGS, offsets)):
            iv = data[ssp_ms][cpu_label]
            if iv.size == 0:
                continue
            all_iv_parts.append(iv)
            pos = gc + offset
            bp = ax.boxplot(
                [iv],
                positions=[pos],
                patch_artist=True,
                widths=BOX_WIDTH,
                notch=False,
                whis=1.5,
                showfliers=False,
                medianprops=dict(color="black", linewidth=1.5),
                whiskerprops=dict(linewidth=1.0, linestyle="-"),
                capprops=dict(linewidth=1.0),
                boxprops=dict(linewidth=0.7),
            )
            for patch in bp["boxes"]:
                patch.set_facecolor(CPU_COLORS[cpu_label])
                patch.set_alpha(0.65)

    # x-axis: one tick per SSP group, labeled in seconds
    ax.set_xticks(group_centers)
    ax.set_xticklabels([f"{ssp / 1000:.4g}" for ssp in SSP_MS_LIST])
    ax.set_xlim(group_centers[0] - 0.55, group_centers[-1] + 0.55)
    ax.set_xlabel("SSP (s)")
    ax.set_ylabel("Inter-message interval (ms)")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    if all_iv_parts:
        all_iv = np.concatenate(all_iv_parts)
        ax.set_ylim(0, np.percentile(all_iv, 99) * 1.10)

    # ── legend ─────────────────────────────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor=COLOR_BASELINE, alpha=0.4, edgecolor="none",
                       label="No attestation"),
    ] + [
        mpatches.Patch(facecolor=CPU_COLORS[lbl], alpha=0.65, edgecolor="black",
                       linewidth=0.7,
                       label=f"CPU = {lbl}" if lbl != "NC" else "CPU = uncapped")
        for lbl, _ in CPU_CONFIGS
    ]
    ax.legend(handles=legend_handles, loc="upper left", frameon=True,
              framealpha=0.9, fancybox=False, edgecolor="black",
              borderpad=0.4, handlelength=1.2, ncol=1)

    out_path = script_dir / "boxplot_ros_topic_hz.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
