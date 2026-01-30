#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

# ---- Config ----
# We use the RAW data file, not the summary, for CDFs
INPUT_FILE = "../data/attestation-times/_summary/all_raw_attestation_durations.csv"
OUTPUT_FILE = "./cdfplot_attestation_cycle_time_per_robot.pdf"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
LINE_WIDTH = 2.0
FIG_SIZE = (7, 4.5)

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}\nDid you run the updated summarizer?")

    df = pd.read_csv(csv_path)

    # CDF requires the raw duration column
    required = {"participant", "duration_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    # Sort Robot1, Robot2, ... logically
    df = df.sort_values(
        "participant",
        key=lambda s: s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    # Theme + palette (Matching barplot style)
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    ordered_labels = df["participant"].unique().tolist()
    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Plot CDF
    sns.ecdfplot(
        data=df,
        x="duration_s",
        hue="participant",
        palette=color_map,
        linewidth=LINE_WIDTH,
        alpha=0.9,
        ax=ax
    )

    # Labels and Title
    ax.set_ylabel(f"CDF attestation cycle time")
    ax.set_xlabel("Time (s)")

    # 1. Add a bit of space at the top (0.0 to 1.05)
    ax.set_ylim(0, 1.05)

    # Grid (Matching barplot style: Y-axis only, dashed)
    ax.set_axisbelow(True)
    ax.grid(axis="both", linestyle="-", linewidth=1.0, alpha=0.8)

    # Spines (Matching barplot style: Black, thick)
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    sns.move_legend(ax, "lower right", frameon=True, framealpha=0.9, title=None)

    plt.tight_layout()
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()