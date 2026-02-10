#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_durations_summary.csv"
OUTPUT_FILE = "./barplot_attestation_cycle_time_blockchain.pdf"

# Options: "e2e_blockchain", "total_lifecycle", "evidence_blockchain_write"
TARGET_METRIC = "e2e_blockchain"
TARGET_ROLE = "prover"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (7, 4.5)

# Toggle features
SHOW_VALUE_LABELS = True
SHOW_ERROR_BARS = True

# Styling
ERR_COLOR = "black"

def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    
    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    required = {"participant", "mean_s", "std_s", "metric", "role"}
    missing = required - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns: {missing}")
        print(f"      Found columns: {df.columns.tolist()}")
        sys.exit(1)

    # --- FILTERING ---
    df = df[
        (df["role"] == TARGET_ROLE) & 
        (df["metric"] == TARGET_METRIC)
    ].copy()

    if df.empty:
        print(f"[ERR] No data found for role='{TARGET_ROLE}' and metric='{TARGET_METRIC}'")
        sys.exit(1)

    # Sort Robot1, Robot2...
    def sort_key(s):
        # Extract number if "RobotX", else put at end
        val = s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float)
        return val.fillna(999)

    df = df.sort_values(by="participant", key=sort_key).reset_index(drop=True)

    # Theme
    sns.set_theme(context="paper", style="ticks", 
                  rc={"xtick.direction": "in", "ytick.direction": "in"}, 
                  font_scale=FONT_SCALE)
    
    ordered_labels = df["participant"].astype(str).tolist()
    # Use colorblind palette
    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    bar_edge_color = "black" if SHOW_ERROR_BARS else None
    bar_line_width = SPINES_WIDTH if SHOW_ERROR_BARS else 0

    # Bars - UPDATED y="mean_s"
    sns.barplot(
        data=df,
        x="participant",
        y="mean_s",
        hue="participant",
        palette=color_map,
        legend=False,
        errorbar=None, 
        edgecolor=bar_edge_color,
        linewidth=bar_line_width,
        ax=ax
    )

    # Y-limit calculation - UPDATED columns
    if SHOW_ERROR_BARS:
        high_points = df["mean_s"] + df["std_s"].fillna(0)
    else:
        high_points = df["mean_s"]

    y_max = high_points.max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = 1.0
    
    ax.set_ylim(0, y_max * 1.20)

    # Error bars + labels
    for i in range(len(df)):
        mean = float(df.loc[i, "mean_s"])
        std = float(df.loc[i, "std_s"])

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

        if SHOW_VALUE_LABELS:
            base_y = (mean + std) if SHOW_ERROR_BARS else mean
            y_text = base_y + 0.02 * ax.get_ylim()[1]

            ax.text(
                i,
                y_text,
                f"{mean:.2f}",
                ha="center",
                va="bottom",
                color=ERR_COLOR,
                fontsize=12,
                zorder=20,
                clip_on=False,
            )

    # Dynamic Label based on metric
    label_map = {
        "e2e_blockchain": "Attestation Cycle Time (s)\nblockchain contribution",
        "total_lifecycle": "Total Lifecycle Duration (s)",
        "evidence_blockchain_write": "Blockchain Write Time (s)"
    }
    y_label = label_map.get(TARGET_METRIC, f"{TARGET_METRIC} (s)")

    ax.set_ylabel(y_label)
    ax.set_xlabel("prover perspective")

    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    out_path = script_dir / OUTPUT_FILE
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)

if __name__ == "__main__":
    main()