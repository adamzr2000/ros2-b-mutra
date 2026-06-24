#!/usr/bin/env python3
"""
Tamper detection — Time to Detection line plot.

For each SSP value: mean detection latency (filled marker + line),
min–max range (error bar). X positions are evenly spaced (categorical)
so all SSP values are readable regardless of their numeric spread.

Data: experiments/data/tamper-detection/results/state_publisher/SSP*ms-batch*.csv

Usage:
  cd experiments/plot
  python3 lineplot_tamper_detection.py
"""

import csv
import math
import statistics
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np
import seaborn as sns

# ── Paths ─────────────────────────────────────────────────────────────────────
SCRIPT_DIR = Path(__file__).parent.resolve()
DATA_DIR   = SCRIPT_DIR.parent / "data" / "tamper-detection" / "results" / "state_publisher"

# ── Style — aligned with barplot_attestation_time_v5_combined.py ─────────────
FONT_SCALE = 1.7
C_DATA     = "#3BA774"   # single dark green for line, markers, and error bars


# ── Data loading ──────────────────────────────────────────────────────────────
def load_data() -> dict[int, list[float]]:
    """Return {ssp_ms: [latency_s, ...]} for detected trials only."""
    data: dict[int, list[float]] = {}
    for f in sorted(DATA_DIR.glob("SSP*ms-batch*.csv")):
        with f.open(encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                if row["detected"] == "1" and row["latency_s"]:
                    ssp = int(row["ssp_ms"])
                    data.setdefault(ssp, []).append(float(row["latency_s"]))
    return data


# ── Plot ──────────────────────────────────────────────────────────────────────
def main() -> None:
    raw = load_data()
    if not raw:
        print(f"[ERR] No data found in {DATA_DIR}")
        return

    ssps_ms  = sorted(raw)
    ssps_s   = [s / 1000.0 for s in ssps_ms]
    means    = np.array([statistics.mean(raw[s])   for s in ssps_ms])
    mins     = np.array([min(raw[s])                for s in ssps_ms])
    maxs     = np.array([max(raw[s])                for s in ssps_ms])
    stds     = np.array([statistics.stdev(raw[s]) if len(raw[s]) > 1 else 0.0
                         for s in ssps_ms])
    ns       = [len(raw[s]) for s in ssps_ms]

    # Decreasing SSP order: curve descends left→right toward the blockchain floor,
    # visually showing the irreducible confirmation cost that SSP reduction cannot beat.
    ssps_ms  = list(reversed(ssps_ms))
    ssps_s   = list(reversed(ssps_s))
    means    = means[::-1]
    mins     = mins[::-1]
    maxs     = maxs[::-1]

    x_pos  = np.arange(len(ssps_ms), dtype=float)
    labels = [str(int(s)) if s == int(s) else str(s) for s in ssps_s]

    # ── Seaborn / Matplotlib setup ────────────────────────────────────────────
    sns.set_theme(
        context="paper", style="ticks",
        rc={"xtick.direction": "out", "ytick.direction": "out"},
        font_scale=FONT_SCALE,
    )
    plt.rcParams.update({"font.family": "serif", "pdf.fonttype": 42, "ps.fonttype": 42})
    

    fig, ax = plt.subplots(figsize=(7.0, 4.2))
    plt.subplots_adjust(left=0.11, right=0.98, top=0.98, bottom=0.12)

    # ── Min–max error bars ────────────────────────────────────────────────────
    ax.errorbar(
        x_pos, means,
        yerr=[means - mins, maxs - means],
        fmt="none",
        color=C_DATA, linewidth=1.5, capsize=5, capthick=1.5, zorder=4,
    )

    # ── Connecting line ───────────────────────────────────────────────────────
    ax.plot(x_pos, means,
            color=C_DATA, linewidth=2.0, zorder=3)

    # ── Filled mean markers ───────────────────────────────────────────────────
    ax.plot(x_pos, means, "o",
            color=C_DATA, markersize=8,
            markerfacecolor=C_DATA,
            markeredgecolor=C_DATA,
            markeredgewidth=1.6,
            zorder=5, label="Mean")

    # ── Value labels above each error-bar cap ─────────────────────────────────
    label_fs = plt.rcParams["font.size"] * 0.8
    y_range  = maxs[0] * 1.12          # approximate plot height used for offset scaling
    v_offset = y_range * 0.025         # ~2.5 % of plot height above the cap

    for xi, (me, mx) in enumerate(zip(means, maxs)):
        ax.text(xi, mx + v_offset, f"{me:.1f}",
                ha="center", va="bottom", fontsize=label_fs,
                color="#222222", zorder=6)

    # ── Axes ──────────────────────────────────────────────────────────────────
    ax.set_xlabel("SSP (s)")
    ax.set_ylabel("Time to Detection (s)")

    ax.set_xticks(x_pos)
    ax.set_xticklabels(labels)
    ax.set_xlim(-0.5, x_pos[-1] + 0.5)

    # Y from 0; top rounded up to nearest 10 with a small headroom for labels.
    y_top = math.ceil(maxs[0] * 1.12 / 10) * 10
    ax.set_ylim(0, y_top)
    ax.set_yticks(range(0, y_top + 1, 10))

    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="both", which="major", linestyle="-", linewidth=0.7, alpha=0.75)

    ax.set_axisbelow(True)

    # ── Legend ────────────────────────────────────────────────────────────────
    from matplotlib.lines import Line2D
    from matplotlib.legend_handler import HandlerBase

    class _ErrorBarHandler(HandlerBase):
        """Draws a vertical stem with horizontal caps — matches the plot's error bars."""
        def create_artists(self, legend, handle, xd, yd, width, height, fontsize, trans):
            cx, cap_w, lw = width / 2, width * 0.32, 1.5
            stem  = Line2D([cx, cx],       [0, height],   color=C_DATA, lw=lw, transform=trans)
            t_cap = Line2D([cx-cap_w, cx+cap_w], [height, height], color=C_DATA, lw=lw, transform=trans)
            b_cap = Line2D([cx-cap_w, cx+cap_w], [0, 0],           color=C_DATA, lw=lw, transform=trans)
            return [stem, t_cap, b_cap]

    mean_handle = Line2D([0], [0], color=C_DATA, linewidth=2.0,
                         marker="o", markersize=8,
                         markerfacecolor=C_DATA, markeredgecolor=C_DATA,
                         label="Mean")
    eb_handle   = Line2D([], [], label="Min–max range")   # proxy; drawn by handler

    ax.legend(
        handles=[mean_handle, eb_handle],
        handler_map={eb_handle: _ErrorBarHandler()},
        loc="upper right",
        frameon=True, framealpha=0.9, fancybox=False,
        edgecolor="black", borderpad=0.5, handlelength=1.4, labelspacing=0.4,
        fontsize=label_fs,
    )
    # ── Console summary ───────────────────────────────────────────────────────
    print(f"\n{'SSP(s)':<8} {'n':>4} {'min':>7} {'mean':>7} {'max':>7} {'std':>7}")
    print("-" * 46)
    for s_ms, s_s, mn, me, mx, sd, n in zip(
            ssps_ms, ssps_s, mins, means, maxs, stds, ns):
        print(f"{s_s:<8.0f} {n:>4} {mn:>7.3f} {me:>7.3f} {mx:>7.3f} {sd:>7.3f}")

    # ── Save ──────────────────────────────────────────────────────────────────
    out = SCRIPT_DIR / "lineplot_tamper_detection.pdf"
    fig.savefig(out, dpi=300, bbox_inches="tight")
    plt.close(fig)
    print(f"\n[OK] Saved: {out.relative_to(SCRIPT_DIR.parent)}")


if __name__ == "__main__":
    main()
