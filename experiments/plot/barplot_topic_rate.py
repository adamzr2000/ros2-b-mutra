#!/usr/bin/env python3
"""
Bar chart: topic publish rate across attestation conditions.

Columns : topics present in the summary CSV (e.g. scan, odom)
X-axis  : attestation condition (No Attestation / Continuous Attestation)
Bar     : median publish rate (Hz); error bars = P25–P75 (IQR), asymmetric
Reference line: nominal publish rate per topic.
"""

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import matplotlib.lines as mlines
import seaborn as sns

INPUT_FILE = "../data/performance-benchmark/summary_topic_stats.csv"

FONT_SCALE = 1.4
BAR_WIDTH  = 0.55
HEADROOM   = 1.18

CONDITION_ORDER = [
    ("no_sidecar",   "baseline"),
    ("with_sidecar", "continuous"),
]
CONDITION_LABELS = [
    "No\nAttestation",
    "Continuous\nAttestation",
]
COLORS = ["#555555", "#6B3FA0"]   # gray / deep purple

# Nominal publish rates per topic for reference lines (Hz); add more as needed
NOMINAL_HZ = {"scan": 5.0, "odom": 30.0}
TOPIC_DISPLAY = {"scan": "LiDAR", "odom": "Odometry"}


def _draw_panel(ax, medians, lows, highs, ylabel, ref_val=None):
    """Bar = median; asymmetric error bars = P25 (down) and P75 (up). Omitted when lows==highs==medians."""
    for i, (med, lo, hi, color) in enumerate(zip(medians, lows, highs, COLORS)):
        ax.bar(i, med, width=BAR_WIDTH, color=color,
               edgecolor="black", linewidth=0.8, zorder=3)
        if lo != med or hi != med:
            ax.errorbar(i, med,
                        yerr=[[med - lo], [hi - med]],
                        fmt="none", color="black",
                        capsize=4, linewidth=1.2, zorder=5)
    ax.set_xticks([])
    ax.set_ylim(0, max(highs) * HEADROOM)
    ax.set_xlim(-0.6, len(medians) - 0.4)
    ax.set_ylabel(ylabel)
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)
    if ref_val is not None:
        ax.axhline(ref_val, color="gray", linewidth=1.0, linestyle="--", zorder=2)


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df     = pd.read_csv(csv_path)
    topics = sorted(df["topic"].unique())
    n_top  = len(topics)

    if n_top == 0:
        raise SystemExit("No topic data found in CSV.")

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, axes = plt.subplots(
        1, n_top,
        figsize=(5.2 * n_top, 4.0),
        gridspec_kw={"wspace": 0.32},
    )
    if n_top == 1:
        axes = [axes]

    for col, topic in enumerate(topics):
        hz_med, hz_p25, hz_p75 = [], [], []

        for cond, mode in CONDITION_ORDER:
            sub = df[(df["topic"] == topic) &
                     (df["condition"] == cond) &
                     (df["mode"] == mode)]
            if sub.empty:
                print(f"[WARN] No data for {topic}/{cond}/{mode} — using 0.")
                hz_med.append(0.0); hz_p25.append(0.0); hz_p75.append(0.0)
            else:
                hz_med.append(float(sub["hz_median"].iloc[0]))
                hz_p25.append(float(sub["hz_p25"].iloc[0]))
                hz_p75.append(float(sub["hz_p75"].iloc[0]))

        print(f"\n[{topic}]")
        for i, (cond, mode) in enumerate(CONDITION_ORDER):
            print(f"  {CONDITION_LABELS[i].replace(chr(10),' '):<28} "
                  f"hz={hz_med[i]:.3f} [{hz_p25[i]:.3f}–{hz_p75[i]:.3f}]")

        ax = axes[col]
        _draw_panel(ax, hz_med, hz_p25, hz_p75,
                    ylabel="Publish rate (Hz)",
                    ref_val=NOMINAL_HZ.get(topic))
        ax.set_title(TOPIC_DISPLAY.get(topic, topic), pad=7)

    # ── shared legend (bottom centre) ─────────────────────────────────────────
    legend_handles = [
        mpatches.Patch(facecolor=c, edgecolor="black", label=lbl.replace("\n", " "))
        for c, lbl in zip(COLORS, CONDITION_LABELS)
    ] + [
        mlines.Line2D([], [], color="gray", linewidth=1.0, linestyle="--",
                      label="Nominal publish rate")
    ]
    fig.legend(handles=legend_handles,
               loc="lower center", bbox_to_anchor=(0.5, -0.06),
               ncol=3, frameon=True, framealpha=0.9, fancybox=True,
               fontsize=plt.rcParams.get("legend.fontsize", 9))

    plt.subplots_adjust(top=0.95, bottom=0.14)

    out_path = script_dir / "barplot_topic_rate.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
