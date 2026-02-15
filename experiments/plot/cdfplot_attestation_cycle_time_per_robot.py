#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_durations_raw.csv"
OUTPUT_FILE = "./cdfplot_attestation_cycle_time_per_robot.pdf"

TARGET_METRIC = "total_lifecycle"
TARGET_ROLE = "prover"  # <--- ADDED: Must isolate the Prover's perspective!

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
LINE_WIDTH = 2.5
FIG_SIZE = (7, 4.5)

def main():
    script_dir = Path(__file__).parent.resolve()
    # Try multiple paths to find the file
    csv_path = (script_dir / INPUT_FILE).resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        print("Did you run the updated 'summarize_attestation_durations.py' first?")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    # Filter for the specific metric AND role
    df = df[
        (df["metric"] == TARGET_METRIC) & 
        (df["role"] == TARGET_ROLE)
    ].copy()

    if df.empty:
        print(f"[ERR] No data found for metric: {TARGET_METRIC} and role: {TARGET_ROLE}")
        sys.exit(1)

    # Sort Logic
    def sort_key(s):
        val = s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float)
        return val.fillna(999)

    df = df.sort_values(by="participant", key=sort_key).reset_index(drop=True)

    # Theme
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "in", "ytick.direction": "in"},
                  font_scale=FONT_SCALE)

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

    # Labels
    ax.set_ylabel("CDF")
    ax.set_xlabel("Attestation Cycle Time (s)")
    # ax.set_title(f"CDF of {TARGET_METRIC.replace('_', ' ').title()}")

    # Limits & Grid
    ax.set_ylim(0, 1.05)
    ax.set_axisbelow(True)
    ax.grid(axis="both", linestyle="--", linewidth=0.5, alpha=0.8)

    # Spines
    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    # Legend
    sns.move_legend(ax, "lower right", frameon=True, framealpha=0.9, title=None)

    plt.tight_layout()
    out_path = script_dir / OUTPUT_FILE
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()