#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import re

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_duration_per_participant.csv"
OUTPUT_FILE = "./boxplot_attestation_cycle_time_per_robot.pdf"

FONT_SCALE = 1.5
LINE_WIDTH = 1.0
SPINES_WIDTH = 1.0
FIG_SIZE = (7, 4.5)

# STROKE_COLOR = "0.2"
STROKE_COLOR = "black"

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # Required columns for boxplot from summary
    required = {"participant", "min_s", "p25_s", "median_s", "p75_s", "max_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns for boxplot: {missing}")

    # Sort robots numerically
    df = df.sort_values(
        "participant",
        key=lambda s: s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    # Theme + palette (match your current paper style)
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    ordered_labels = df["participant"].astype(str).tolist()
    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Manually draw the boxes for each robot
    for i, row in df.iterrows():
        label = str(row["participant"])
        face_c = color_map[label]

        # Stats
        low = float(row["min_s"])
        q1 = float(row["p25_s"])
        med = float(row["median_s"])
        q3 = float(row["p75_s"])
        high = float(row["max_s"])

        # 1) Whiskers
        ax.vlines(i, low, q1, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)
        ax.vlines(i, q3, high, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)

        # 2) Whisker caps
        cap_w = 0.22
        ax.hlines(low,  i - cap_w / 2, i + cap_w / 2, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)
        ax.hlines(high, i - cap_w / 2, i + cap_w / 2, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)

        # 3) Box (colored fill, neutral outline)
        box_w = 0.60
        rect = plt.Rectangle(
            (i - box_w / 2, q1),
            box_w,
            q3 - q1,
            facecolor=face_c,
            edgecolor=STROKE_COLOR,
            lw=LINE_WIDTH,
            zorder=3,
        )
        ax.add_patch(rect)

        # 4) Median line (neutral)
        ax.hlines(med, i - box_w / 2, i + box_w / 2, color=STROKE_COLOR, lw=LINE_WIDTH * 1.5, zorder=4)

    # Formatting
    ax.set_xlim(-0.5, len(df) - 0.5)
    ax.set_xticks(range(len(df)))
    ax.set_xticklabels(df["participant"])

    y_max = df["max_s"].max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = 1.0
    ax.set_ylim(0, y_max * 1.25)

    ax.set_ylabel("Attestation Cycle Time (s)")

    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved boxplot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()
