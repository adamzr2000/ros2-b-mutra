#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_duration_per_participant.csv"
OUTPUT_FILE = "./barplot_attestation_cycle_time_per_robot.pdf"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (7, 4.5)

# Toggle features
SHOW_VALUE_LABELS = True
SHOW_ERROR_BARS = True

# Error bars + value labels styling
# ERR_COLOR = "0.2"
ERR_COLOR = "black"

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    required = {"participant", "mean_of_means_s", "std_of_means_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    # Sort Robot1, Robot2, ...
    df = df.sort_values(
        "participant",
        key=lambda s: s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    # Theme + palette
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)
    ordered_labels = df["participant"].astype(str).tolist()
    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Determine edge style based on flags
    # If we are showing error bars (scientific style), we add a black outline to the bars.
    # Otherwise, we keep them flat.
    bar_edge_color = "black" if SHOW_ERROR_BARS else None
    bar_line_width = SPINES_WIDTH if SHOW_ERROR_BARS else 0

    # Bars
    sns.barplot(
        data=df,
        x="participant",
        y="mean_of_means_s",
        hue="participant",
        palette=color_map,
        legend=False,
        errorbar=None,  # add std manually
        edgecolor=bar_edge_color, # <--- Added conditional edge
        linewidth=bar_line_width, # <--- Added conditional width
        ax=ax,
    )

    # Y-limit headroom calculation
    if SHOW_ERROR_BARS:
        high_points = df["mean_of_means_s"] + df["std_of_means_s"].fillna(0)
    else:
        high_points = df["mean_of_means_s"]

    y_max = high_points.max()
    
    if pd.isna(y_max) or y_max <= 0:
        y_max = max(1.0, df["mean_of_means_s"].max() if len(df) else 1.0)
    
    ax.set_ylim(0, y_max * 1.20)

    # Error bars + labels
    for i in range(len(df)):
        mean = float(df.loc[i, "mean_of_means_s"])
        std = float(df.loc[i, "std_of_means_s"])

        # 1. Draw Error Bars (if enabled)
        if SHOW_ERROR_BARS:
            ax.errorbar(
                x=i,
                y=mean,
                yerr=std,
                fmt="none",
                ecolor=ERR_COLOR,
                elinewidth=1.0,
                capsize=4,
                capthick=1.0,
                zorder=10,
            )

        # 2. Draw Value Labels (if enabled)
        if SHOW_VALUE_LABELS:
            base_y = (mean + std) if SHOW_ERROR_BARS else mean
            y_text = base_y + 0.015 * ax.get_ylim()[1]
            
            ax.text(
                i,
                y_text,
                f"{mean:.2f}",
                ha="center",
                va="bottom",
                color=ERR_COLOR,
                fontsize=plt.rcParams["font.size"],
                zorder=20,
                clip_on=False,
            )

    ax.set_ylabel("Attestation Cycle Time (s)")
    ax.set_xlabel("")

    ax.set_axisbelow(True)
    # Only vertical grid for bar plots is usually cleaner
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()