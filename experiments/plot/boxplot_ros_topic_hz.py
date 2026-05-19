#!/usr/bin/env python3
"""
Two-panel boxplot: /tf inter-message interval distribution vs SSP and CPU limit.

Uses ros_stamp_ns (publisher-side clock) — immune to DDS batching artefacts.
All robots are pooled with equal weight (~36 k intervals per condition).

Left  — SSP sweep : CPU = 1.0 fixed, SSP ∈ {20 5 1 0.5 0.1} s
Right — CPU sweep : SSP = 0.1 s fixed, CPU ∈ {1.0 0.5 0.25 0.1}

Box  : Q1 – Q3  (IQR)
Line : median
Whiskers : Q1 − 1.5×IQR  to  Q3 + 1.5×IQR  (Tukey standard)
Fliers   : hidden (36 k points → too many dots)
Baseline : gray IQR band + dashed median line (no redundant box on x-axis)
"""

import re
from pathlib import Path

import numpy as np
import matplotlib.patches as mpatches
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

RESULTS_DIR = Path(__file__).parent.parent / "data/robot-stats/results/tf"
NOMINAL_MS  = 1000.0 / 30.0   # ~33.3 ms
FONT_SCALE  = 1.7

_TAGGED_RE = re.compile(
    r"SSP(?P<ssp>\d+)ms-ITERQ\d+-cpu(?P<cpu>[\dp]+|NC)-run\d+\.csv$",
    re.IGNORECASE,
)

COLOR_BASELINE = "#888888"
COLOR_SIDECAR  = "#6B3FA0"


def _intervals_ms(csv_path: Path) -> np.ndarray:
    """
    Per-robot inter-message intervals (ms) from the collector wall clock.
    Computed within each robot's own message sequence so cross-robot DDS
    batching artefacts never appear (same-robot messages arrive sequentially).
    """
    df = pd.read_csv(csv_path)
    parts = []
    for _, grp in df.groupby("robot"):
        grp = grp.sort_values("wall_stamp_ns")
        iv = grp["wall_stamp_ns"].diff().dropna().values / 1e6   # ns → ms
        parts.append(iv[iv > 0])
    return np.concatenate(parts) if parts else np.array([])


def _collect_baseline(results_tf: Path) -> np.ndarray:
    csvs = sorted((results_tf / "no_sidecar" / "baseline").rglob("*.csv"))
    parts = [_intervals_ms(p) for p in csvs]
    return np.concatenate(parts) if parts else np.array([])


def _collect_sidecar(results_tf: Path, ssp_ms=None, cpu=None) -> np.ndarray:
    """cpu: float for a specific limit, "NC" for uncapped, None to collect all."""
    parts = []
    for p in sorted((results_tf / "with_sidecar" / "continuous").rglob("*.csv")):
        m = _TAGGED_RE.search(p.name)
        if not m:
            continue
        if ssp_ms is not None and int(m.group("ssp")) != ssp_ms:
            continue
        if cpu is not None:
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
    print(f"[{label}]  n={iv.size:,}  "
          f"median={med:.2f} ms  IQR=[{q1:.2f}, {q3:.2f}]  "
          f"std={iv.std():.2f} ms  p95={np.percentile(iv, 95):.2f} ms")


def _add_baseline_band(ax, iv: np.ndarray) -> None:
    if iv.size == 0:
        return
    q1, med, q3 = np.percentile(iv, [25, 50, 75])
    ax.axhspan(q1, q3, color=COLOR_BASELINE, alpha=0.18, zorder=1)
    ax.axhline(med, color=COLOR_BASELINE, linewidth=1.4, linestyle="--", zorder=2)


def _draw_boxes(ax, groups) -> None:
    """groups: [(x_label, intervals_ms_array), ...]"""
    xs   = list(range(len(groups)))
    data = [g[1] for g in groups]

    bp = ax.boxplot(
        data,
        positions=xs,
        patch_artist=True,
        widths=0.52,
        notch=False,
        whis=1.5,           # standard Tukey: Q1-1.5*IQR to Q3+1.5*IQR
        showfliers=False,   # 36k points → fliers would be visual noise
        medianprops=dict(color="black", linewidth=1.8),
        whiskerprops=dict(linewidth=1.2, linestyle="-"),
        capprops=dict(linewidth=1.2),
        boxprops=dict(linewidth=0.8),
    )
    for patch in bp["boxes"]:
        patch.set_facecolor(COLOR_SIDECAR)
        patch.set_alpha(0.6)

    ax.set_xticks(xs)
    ax.set_xticklabels([g[0] for g in groups])
    ax.set_xlim(-0.62, len(xs) - 0.38)
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)


def main():
    script_dir = Path(__file__).parent.resolve()

    if not RESULTS_DIR.exists():
        raise SystemExit(f"Results dir not found: {RESULTS_DIR}\nRun the benchmark first.")

    iv_baseline = _collect_baseline(RESULTS_DIR)
    _print_stats("baseline", iv_baseline)

    # ── SSP sweep (no CPU cap) ────────────────────────────────────────────────
    SSP_MS_LIST = [5000, 1000, 500, 100, 10]
    ssp_groups = []
    for ssp in SSP_MS_LIST:
        iv    = _collect_sidecar(RESULTS_DIR, ssp_ms=ssp, cpu="NC")
        label = f"{ssp / 1000:.4g}"    # "5", "1", "0.5", "0.1", "0.01"
        _print_stats(f"SSP={label}s CPU=NC", iv)
        ssp_groups.append((label, iv))

    # ── CPU sweep (SSP = 100 ms = 0.1 s) ─────────────────────────────────────
    CPU_VALS = [1.0, 0.5, 0.25, 0.1]
    cpu_groups = []
    for cpu in CPU_VALS:
        iv    = _collect_sidecar(RESULTS_DIR, ssp_ms=10, cpu=cpu)
        label = str(cpu)
        _print_stats(f"SSP=0.01s CPU={label}", iv)
        cpu_groups.append((label, iv))

    # ── Plot ──────────────────────────────────────────────────────────────────
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, (ax_ssp, ax_cpu) = plt.subplots(
        1, 2, figsize=(10.0, 4.4),
        sharey=True,
        gridspec_kw={"wspace": 0.08},
    )

    for ax in (ax_ssp, ax_cpu):
        _add_baseline_band(ax, iv_baseline)

    _draw_boxes(ax_ssp, ssp_groups)
    ax_ssp.set_ylabel("Inter-message interval (ms)")
    ax_ssp.set_xlabel("SSP (s)   [no CPU cap]", labelpad=6)
    ax_ssp.set_title("SSP sweep", pad=7)

    _draw_boxes(ax_cpu, cpu_groups)
    ax_cpu.set_xlabel("CPU limit   [SSP = 0.01 s]", labelpad=6)
    ax_cpu.set_title("CPU sweep", pad=7)

    # y limits: 0 to 99th percentile of all data + padding
    all_iv = np.concatenate(
        [iv_baseline] + [g[1] for g in ssp_groups + cpu_groups if g[1].size > 0]
    )
    if all_iv.size:
        p99 = np.percentile(all_iv, 99)
        ax_ssp.set_ylim(0, p99 * 1.10)

    legend_handles = [
        mpatches.Patch(facecolor=COLOR_BASELINE, alpha=0.4, edgecolor="none",
                       label="No attestation  (IQR band + median)"),
        mpatches.Patch(facecolor=COLOR_SIDECAR, alpha=0.6, edgecolor="black",
                       linewidth=0.8,
                       label="With attestation  (box=IQR, whiskers=1.5×IQR)"),
    ]
    fig.legend(
        handles=legend_handles,
        loc="lower center", bbox_to_anchor=(0.5, -0.04),
        ncol=2, frameon=True, framealpha=0.9,
        fancybox=False, edgecolor="black", borderpad=0.4,
    )

    plt.subplots_adjust(top=0.93, bottom=0.24)

    out_path = script_dir / "boxplot_ros_topic_hz.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
