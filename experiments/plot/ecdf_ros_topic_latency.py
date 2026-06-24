#!/usr/bin/env python3
"""
ECDF: ROS topic inter-message interval — SSP sweep vs. no-attestation baseline.

Usage:
  python3 ecdf_ros_topic_latency.py [--topic TOPIC]

  --topic TOPIC   Topic subfolder name under results/ (default: tf)

Each curve is the ECDF of wall-clock inter-arrival times (ms).
CPU configs are pooled per SSP — CPU has no measurable effect.
X-axis is zoomed to the p0.1–p99.9 range of the combined data to expose
any distributional shift; overlapping curves confirm operational transparency.
"""

import argparse
import re
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import pandas as pd
import seaborn as sns

RESULTS_BASE = Path(__file__).parent.parent / "data/robot-stats/results"
FONT_SCALE   = 1.7

SSP_MS_LIST  = [10000, 5000, 1000, 10]   # 5 s, 1 s, 0.1 s, 0.01 s
SSP_LABELS   = ["10", "5", "1", "0.01"]

# Sequential blue ramp — light→dark maps SSP 5s→0.01s (slow→fast polling).
# Avoids semantic collision with paper palette (green=robot, purple=SECaaS,
# orange=oracle, dark-blue=#4B9FCC=SHA-256 in barplot).
_COLORS = ["#bdd7e7", "#6baed6", "#2171b5", "#084594"]   # ColorBrewer Blues-4
SSP_STYLES = [
    dict(color=_COLORS[0], linewidth=2.0, linestyle="solid"),
    dict(color=_COLORS[1], linewidth=2.0, linestyle="solid"),
    dict(color=_COLORS[2], linewidth=2.0, linestyle="solid"),
    dict(color=_COLORS[3], linewidth=2.0, linestyle="solid"),
]
BASELINE_STYLE = dict(color="#888888", linewidth=2.5, linestyle="solid")

_TAGGED_RE = re.compile(
    r"SSP(?P<ssp>\d+)ms-ITERQ\d+-cpu(?P<cpu>[\dp]+|NC)-run\d+\.csv$",
    re.IGNORECASE,
)


def _intervals_ms(csv_path: Path) -> np.ndarray:
    df = pd.read_csv(csv_path)
    parts = []
    for _, grp in df.groupby("robot"):
        grp = grp.sort_values("wall_stamp_ns")
        iv = grp["wall_stamp_ns"].diff().dropna().values / 1e6
        parts.append(iv[iv > 0])
    return np.concatenate(parts) if parts else np.array([])


def _collect_baseline(results_dir: Path) -> np.ndarray:
    csvs = sorted((results_dir / "no_sidecar" / "baseline").rglob("*.csv"))
    parts = [_intervals_ms(p) for p in csvs]
    return np.concatenate(parts) if parts else np.array([])


def _collect_sidecar(results_dir: Path, ssp_ms: int) -> np.ndarray:
    parts = []
    for p in sorted((results_dir / "with_sidecar" / "continuous").rglob("*.csv")):
        m = _TAGGED_RE.search(p.name)
        if m and int(m.group("ssp")) == ssp_ms:
            parts.append(_intervals_ms(p))
    return np.concatenate(parts) if parts else np.array([])


def _ecdf(iv: np.ndarray):
    xs = np.sort(iv)
    ys = np.arange(1, len(xs) + 1) / len(xs)
    return xs, ys


def _print_stats(label: str, iv: np.ndarray) -> None:
    if iv.size == 0:
        print(f"[{label}]  no data")
        return
    p25, p50, p75, p80, p90, p95, p99 = np.percentile(iv, [25, 50, 75, 80, 90, 95, 99])
    print(f"[{label}]  n={iv.size:,}  min={iv.min():.3f}  "
          f"p25={p25:.3f}  p50={p50:.3f}  p75={p75:.3f}  "
          f"p80={p80:.3f}  p90={p90:.3f}  p95={p95:.3f}  p99={p99:.3f}  "
          f"max={iv.max():.3f}  (ms)")


def main():
    parser = argparse.ArgumentParser(description="ECDF ROS topic inter-message interval.")
    parser.add_argument("--topic", default="tf",
                        help="Topic subfolder under results/ (default: tf)")
    args = parser.parse_args()

    topic       = args.topic
    results_dir = RESULTS_BASE / topic
    script_dir  = Path(__file__).parent.resolve()

    if not results_dir.exists():
        raise SystemExit(f"Results dir not found: {results_dir}")

    iv_baseline = _collect_baseline(results_dir)
    _print_stats("baseline", iv_baseline)

    ssp_data = []
    for ssp_ms, label in zip(SSP_MS_LIST, SSP_LABELS):
        iv = _collect_sidecar(results_dir, ssp_ms)
        _print_stats(f"SSP={ssp_ms}ms", iv)
        ssp_data.append((label, iv))

    # ── x-axis zoom: p0.1 – p99.9 of all data combined ────────────────────────
    all_iv = np.concatenate(
        [iv_baseline] + [iv for _, iv in ssp_data if iv.size > 0]
    )
    x_lo = float(iv_baseline.min())
    x_hi = np.percentile(all_iv, 99.9)
    margin = (x_hi - x_lo) * 0.08
    x_hi += margin

    # ── plot ────────────────────────────────────────────────────────────────────
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})

    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    plt.subplots_adjust(left=0.13, right=0.97, top=0.95, bottom=0.18)

    # baseline
    if iv_baseline.size:
        xs, ys = _ecdf(iv_baseline)
        ax.plot(xs, ys, label="No attestation", zorder=5, **BASELINE_STYLE)

    # one curve per SSP
    for (label, iv), style in zip(ssp_data, SSP_STYLES):
        if iv.size == 0:
            continue
        xs, ys = _ecdf(iv)
        ax.plot(xs, ys, label=f"SSP = {label} s", zorder=4, **style)

    ax.set_xlim(x_lo, x_hi)
    ax.set_ylim(-0.02, 1.05)
    ax.set_xlabel("Robot message inter-arrival time (ms)")
    ax.set_ylabel("ECDF")
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="both", linestyle="-", linewidth=0.6, alpha=0.6)
    ax.set_axisbelow(True)

    ax.legend(loc="lower left", frameon=True, framealpha=0.9,
              fancybox=False, edgecolor="black",
              borderpad=0.5, handlelength=1.8, fontsize="x-small")

    # ── zoom inset: right tail where curves separate ───────────────────────────
    # focus on the p80–p95 region (y = 0.80 – 0.95)
    z_lo = np.percentile(all_iv, 80)
    z_hi = np.percentile(all_iv, 95)
    z_margin = (z_hi - z_lo) * 0.12
    z_lo -= z_margin
    z_hi += z_margin

    # inset in lower-right corner
    axins = ax.inset_axes([0.54, 0.44, 0.44, 0.42])
    if iv_baseline.size:
        xs, ys = _ecdf(iv_baseline)
        axins.plot(xs, ys, zorder=5, **BASELINE_STYLE)
    for (label, iv), style in zip(ssp_data, SSP_STYLES):
        if iv.size == 0:
            continue
        xs, ys = _ecdf(iv)
        axins.plot(xs, ys, zorder=4, **style)

    axins.set_xlim(z_lo, z_hi)
    axins.set_ylim(0.79, 0.96)
    axins.tick_params(labelsize=plt.rcParams["font.size"] * 0.8,
                      length=4, width=0.8, direction="out")
    axins.grid(axis="both", linestyle="-", linewidth=0.5, alpha=0.5)
    axins.set_axisbelow(True)
    axins.xaxis.set_major_locator(plt.MaxNLocator(4))
    axins.yaxis.set_major_locator(plt.MaxNLocator(4))

    # indicate_inset_zoom returns (rectangle, connectors) on older Matplotlib
    # and an InsetIndicator object with a .connectors attribute on 3.10+.
    result = ax.indicate_inset_zoom(
        axins, edgecolor="black", linewidth=1.0, linestyle="--", alpha=0.9
    )
    connectors = result.connectors if hasattr(result, "connectors") else result[1]
    # connectors order: [0] lower-left, [1] upper-left, [2] lower-right, [3] upper-right
    # keep only bottom-left and top-right (diagonal pair)
    for i, conn in enumerate(connectors):
        conn.set_visible(i in (0, 3))
        conn.set_linestyle("--")
        conn.set_color("black")
        conn.set_linewidth(1.0)

    out_path = script_dir / f"ecdf_ros_topic_latency_{topic}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
