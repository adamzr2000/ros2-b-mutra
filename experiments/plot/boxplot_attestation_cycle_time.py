#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys
import re

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_durations_summary.csv"
OUTPUT_FILE = "./boxplot_attestation_cycle_time.pdf"

# Options: "e2e_blockchain", "total_lifecycle", "evidence_tx_confirm", "evidence_call"
TARGET_METRIC = "total_lifecycle"
TARGET_ROLE = "prover"

FONT_SCALE = 1.5
LINE_WIDTH = 1.0
SPINES_WIDTH = 1.0
FIG_SIZE = (7, 4.5)
STROKE_COLOR = "black"

def main():
    # Fix path resolution relative to script location
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()

    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    # Required columns for boxplot from summary
    required = {"participant", "min_s", "p25_s", "median_s", "p75_s", "max_s", "metric", "role"}
    missing = required - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns for boxplot: {missing}")
        sys.exit(1)

    # --- FILTERING ---
    df = df[
        (df["role"] == TARGET_ROLE) & 
        (df["metric"] == TARGET_METRIC)
    ].copy()

    if df.empty:
        print(f"[ERR] No data found for role='{TARGET_ROLE}' and metric='{TARGET_METRIC}'")
        sys.exit(1)

    # Sort robots numerically
    def sort_key(s):
        val = s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float)
        return val.fillna(999)

    df = df.sort_values(by="participant", key=sort_key).reset_index(drop=True)

    # Theme
    sns.set_theme(context="paper", style="ticks", 
                  rc={"xtick.direction": "in", "ytick.direction": "in"}, 
                  font_scale=FONT_SCALE)

    ordered_labels = df["participant"].astype(str).tolist()
    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Manually draw the boxes
    # We iterate by index (0, 1, 2...) for X-axis placement
    for i, row in df.iterrows():
        # Because we reset_index, 'i' corresponds to 0, 1, 2...
        # But let's be safe and use enumerate if needed, though iterrows on reset df is fine.
        
        label = str(row["participant"])
        # Get color based on label
        face_c = color_map[label] if label in color_map else "gray"

        low = float(row["min_s"])
        q1 = float(row["p25_s"])
        med = float(row["median_s"])
        q3 = float(row["p75_s"])
        high = float(row["max_s"])

        # 1) Whiskers (vertical lines)
        ax.vlines(i, low, q1, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)
        ax.vlines(i, q3, high, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)

        # 2) Whisker caps (horizontal lines)
        cap_w = 0.2
        ax.hlines(low,  i - cap_w/2, i + cap_w/2, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)
        ax.hlines(high, i - cap_w/2, i + cap_w/2, color=STROKE_COLOR, lw=LINE_WIDTH, zorder=2)

        # 3) Box (colored fill, black outline)
        box_w = 0.6
        rect = plt.Rectangle(
            (i - box_w/2, q1),
            box_w,
            q3 - q1,
            facecolor=face_c,
            edgecolor=STROKE_COLOR,
            linewidth=LINE_WIDTH,
            zorder=3
        )
        ax.add_patch(rect)

        # 4) Median line
        ax.hlines(med, i - box_w/2, i + box_w/2, color=STROKE_COLOR, lw=LINE_WIDTH * 1.5, zorder=4)

    # Formatting X-Axis
    ax.set_xlim(-0.5, len(df) - 0.5)
    ax.set_xticks(range(len(df)))
    ax.set_xticklabels(df["participant"])

    # Dynamic Y-Axis Limit
    y_max = df["max_s"].max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = 1.0
    ax.set_ylim(0, y_max * 1.15)

    # Dynamic Label
    label_map = {
        "e2e_blockchain": "Attestation Cycle Time (s)\nblockchain contribution",
        "total_lifecycle": "Attestation Cycle Time (s)\nprover perspective",
        "evidence_tx_confirm": "Blockchain Write Time (s)",
        "evidence_call": "Local RPC Call Time (s)"
    }
    y_label = label_map.get(TARGET_METRIC, f"{TARGET_METRIC} (s)")
    
    ax.set_ylabel(y_label)
    ax.set_xlabel("")

    # Grid & Spines
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    out_path = script_dir / OUTPUT_FILE
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved boxplot to: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()