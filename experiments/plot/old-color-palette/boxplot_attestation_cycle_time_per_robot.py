#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import re

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_duration_per_robot.csv"
OUTPUT_FILE = "./attestation_cycle_time_per_robot_boxplot.pdf"

FONT_SCALE = 1.5
LINE_WIDTH = 1.5
SPINES_WIDTH = 1.5
FIG_SIZE = (7, 4.5)

EDGE_COLORS = [
    "#0000FF",  # blue
    "#FF0000",  # red
    "#008000",  # green
    "#FF7F00",  # orange
    "#6A00FF",  # purple
    "#00A6A6",  # teal
]

def to_rgb(hex_color: str):
    hex_color = hex_color.strip()
    if hex_color.startswith("#"):
        hex_color = hex_color[1:]
    r = int(hex_color[0:2], 16) / 255.0
    g = int(hex_color[2:4], 16) / 255.0
    b = int(hex_color[4:6], 16) / 255.0
    return r, g, b

def _lighten(hex_color: str, factor: float = 0.65) -> str:
    r, g, b = to_rgb(hex_color)
    r = r + (1 - r) * factor
    g = g + (1 - g) * factor
    b = b + (1 - b) * factor
    return "#{:02X}{:02X}{:02X}".format(int(r * 255), int(g * 255), int(b * 255))

FACE_COLORS = [_lighten(c, factor=0.7) for c in EDGE_COLORS]

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # Required columns for boxplot from summary
    required = {"robot", "min_s", "p25_s", "median_s", "p75_s", "max_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns for boxplot: {missing}")

    # Sort robots numerically
    df = df.sort_values(
        "robot",
        key=lambda s: s.str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # We manually draw the boxes for each robot
    for i, row in df.iterrows():
        color_idx = i % len(EDGE_COLORS)
        edge_c = EDGE_COLORS[color_idx]
        face_c = FACE_COLORS[color_idx]

        # Stats
        low, q1, med, q3, high = row['min_s'], row['p25_s'], row['median_s'], row['p75_s'], row['max_s']

        # 1. Draw the whiskers (vertical lines)
        ax.vlines(i, low, q1, color=edge_c, lw=LINE_WIDTH)
        ax.vlines(i, q3, high, color=edge_c, lw=LINE_WIDTH)

        # 2. Draw whisker caps (horizontal lines)
        cap_w = 0.2
        ax.hlines(low, i - cap_w/2, i + cap_w/2, color=edge_c, lw=LINE_WIDTH)
        ax.hlines(high, i - cap_w/2, i + cap_w/2, color=edge_c, lw=LINE_WIDTH)

        # 3. Draw the box (rectangle)
        box_w = 0.6
        rect = plt.Rectangle((i - box_w/2, q1), box_w, q3 - q1, 
                             facecolor=face_c, edgecolor=edge_c, lw=LINE_WIDTH, zorder=3)
        ax.add_patch(rect)

        # 4. Draw the median line
        ax.hlines(med, i - box_w/2, i + box_w/2, color=edge_c, lw=LINE_WIDTH, zorder=4)

        # 5. Optional: Add median text label
        # y_offset = 0.02 * (df['max_s'].max())
        # ax.text(i, high + y_offset, f"{med:.2f}", ha='center', va='bottom', 
        #         color=edge_c, fontsize=plt.rcParams["font.size"])

    # Formatting
    ax.set_xlim(-0.5, len(df) - 0.5)
    ax.set_xticks(range(len(df)))
    ax.set_xticklabels(df['robot'])
    
    ax.set_ylim(0, df['max_s'].max() * 1.25)
    ax.set_ylabel("Time (s)")
    ax.set_title("Attestation cycle time per robot", pad=15)
    
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="--", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved boxplot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()