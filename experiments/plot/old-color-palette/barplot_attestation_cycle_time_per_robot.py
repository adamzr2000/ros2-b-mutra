#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_duration_per_robot.csv"
OUTPUT_FILE = "./attestation_cycle_time_per_robot_barplot.pdf"

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

FACE_COLORS = [_lighten(c, factor=0.6) for c in EDGE_COLORS]

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    
    # --- UPDATED COLUMN NAMES ---
    required = {"robot", "mean_of_means_s", "std_of_means_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    df = df.sort_values(
        "robot",
        key=lambda s: s.str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Draw the Bars
    sns.barplot(
        data=df,
        x="robot",
        y="mean_of_means_s", # Updated
        hue="robot",
        palette=FACE_COLORS,
        linewidth=LINE_WIDTH,
        legend=False,
        errorbar=None,
        ax=ax,
    )

    y_max = (df["mean_of_means_s"] + df["std_of_means_s"].fillna(0)).max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = max(1.0, df["mean_of_means_s"].max() if len(df) else 1.0)
    ax.set_ylim(0, y_max * 1.20)

    for i, bar in enumerate(ax.patches):
        current_edge = EDGE_COLORS[i % len(EDGE_COLORS)]
        bar.set_edgecolor(current_edge)

        row = df.iloc[i]
        mean = float(row["mean_of_means_s"]) # Updated
        std = float(row["std_of_means_s"])   # Updated

        ax.errorbar(
            x=i,
            y=mean,
            yerr=std,
            fmt="none",
            ecolor=current_edge,
            elinewidth=LINE_WIDTH,
            capsize=5,
            capthick=LINE_WIDTH,
            zorder=10,
        )

        x_center = bar.get_x() + bar.get_width() / 2.0
        y_top = mean + std
        y_offset = 0.015 * ax.get_ylim()[1]

        ax.text(
            x_center,
            y_top + y_offset,
            f"{mean:.2f}",
            ha="center",
            va="bottom",
            color=current_edge,
            fontsize=plt.rcParams["font.size"],
            zorder=20,
            clip_on=False,
        )

    ax.set_ylabel("Time (s)")
    ax.set_xlabel("")
    ax.set_title("Attestation cycle time per robot", pad=15)
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="--", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()