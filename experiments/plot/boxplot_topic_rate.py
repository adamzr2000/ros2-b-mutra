#!/usr/bin/env python3
"""
Boxplot of per-run median publish rate, per topic.

Each data point is the median Hz across all robots in one run.
Boxes are drawn over the 5 run-median values; individual points are
overlaid so the small sample size is transparent.

Columns : one per topic found under results/ (e.g. scan, odom)
X-axis  : attestation condition (No Attestation / Continuous Attestation)
Colors  : gray / deep purple
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import seaborn as sns

SUMMARY_CSV = Path(__file__).parent.parent / "data/performance-benchmark/summary_topic_stats.csv"

FONT_SCALE = 1.4

CONDITION_ORDER = [
    ("no_sidecar",   "baseline"),
    ("with_sidecar", "continuous"),
]
CONDITION_LABELS = [
    "No Attestation",
    "Continuous Attestation",
]
COLORS = ["#555555", "#6B3FA0"]

NOMINAL_HZ    = {"scan": 5.0, "odom": 30.0}
TOPIC_DISPLAY = {"scan": "LiDAR", "odom": "Odometry"}


def main():
    script_dir = Path(__file__).parent.resolve()

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    if not SUMMARY_CSV.exists():
        raise SystemExit(f"Summary CSV not found: {SUMMARY_CSV}\nRun summarize_topic_stats.py first.")
    summary = pd.read_csv(SUMMARY_CSV)

    topics = sorted(summary["topic"].unique())
    if not topics:
        raise SystemExit("No topics found in summary CSV.")
    n_top = len(topics)

    fig, axes = plt.subplots(1, n_top, figsize=(4.8 * n_top, 4.0),
                             gridspec_kw={"wspace": 0.32})
    if n_top == 1:
        axes = [axes]

    rng = np.random.default_rng(42)

    for col, topic in enumerate(topics):
        ax = axes[col]
        hz_data = []

        for (cond, mode), color in zip(CONDITION_ORDER, COLORS):
            row = summary[(summary["topic"] == topic) &
                          (summary["condition"] == cond) &
                          (summary["mode"] == mode)]
            if row.empty:
                print(f"[WARN] No data for {topic}/{cond}/{mode}")
                hz_data.append(np.array([]))
                continue
            row = row.iloc[0]
            run_cols = sorted(c for c in summary.columns if c.startswith("hz_run"))
            vals = row[run_cols].dropna().to_numpy(dtype=float)
            hz_data.append(vals)
            print(f"[{topic}/{cond}] runs={vals.size}  "
                  f"median={np.median(vals):.3f}  "
                  f"min={vals.min():.3f}  max={vals.max():.3f}")

        xs = list(range(len(CONDITION_ORDER)))

        bp = ax.boxplot(hz_data, positions=xs, patch_artist=True,
                        widths=0.45, notch=False,
                        flierprops=dict(marker=""),   # hide default fliers
                        medianprops=dict(color="black", linewidth=1.8),
                        whiskerprops=dict(linewidth=1.2),
                        capprops=dict(linewidth=1.2),
                        boxprops=dict(linewidth=0.8))
        for patch, color in zip(bp["boxes"], COLORS):
            patch.set_facecolor(color)
            patch.set_alpha(0.6)

        # Overlay individual run points with jitter
        for x, vals, color in zip(xs, hz_data, COLORS):
            if vals.size == 0:
                continue
            jitter = rng.uniform(-0.08, 0.08, size=vals.size)
            ax.scatter(x + jitter, vals, color=color, edgecolors="black",
                       linewidths=0.5, s=30, zorder=4, alpha=0.9)

        ref = NOMINAL_HZ.get(topic)
        if ref is not None:
            ax.axhline(ref, color="gray", linewidth=1.0, linestyle="--", zorder=2)

        ax.set_xticks([])
        ax.set_xlim(-0.6, len(xs) - 0.4)
        ax.set_ylabel("Publish rate (Hz)")
        ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
        ax.grid(axis="y", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)
        ax.set_title(TOPIC_DISPLAY.get(topic, topic), pad=7)

    legend_handles = [
        mpatches.Patch(facecolor=c, edgecolor="black", alpha=0.75, label=lbl)
        for c, lbl in zip(COLORS, CONDITION_LABELS)
    ] + [
        mlines.Line2D([], [], color="gray", linewidth=1.0, linestyle="--",
                      label="Nominal publish rate")
    ]
    fig.legend(handles=legend_handles,
               loc="lower center", bbox_to_anchor=(0.5, -0.04),
               ncol=3, frameon=True, framealpha=0.9, fancybox=True,
               fontsize=plt.rcParams.get("legend.fontsize", 9))

    plt.subplots_adjust(top=0.95, bottom=0.14)

    out_path = script_dir / "boxplot_topic_rate.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
