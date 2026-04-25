#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path
import sys

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/durations_summary.csv"

# Filter: which experiment slice to plot
N_ROBOTS = 4             # e.g. 4 or 8
MODES    = ["startup", "continuous"]  # Generate PDFs for both modes

# Options: "e2e_blockchain", "total_lifecycle", "evidence_tx_confirm", "evidence_call"
TARGET_METRIC = "total_lifecycle"
TARGET_ROLE = "prover"

FONT_SCALE = 2
SPINES_WIDTH = 1.0
FIG_SIZE = (7, 4.5)

# Toggle features
SHOW_VALUE_LABELS = False
SHOW_ERROR_BARS = True

# Styling
ERR_COLOR = "black"
BAR_COLOR = sns.color_palette("tab10", n_colors=10)[0]
VALUE_LABEL_COLOR = "black"

def generate_plot(df, mode, script_dir):
    if df.empty:
        print(f"[ERR] No data found for mode='{mode}', role='{TARGET_ROLE}' and metric='{TARGET_METRIC}'")
        return False

    # Sort Robot1, Robot2...
    def sort_key(s):
        # Extract number if "RobotX", else put at end
        val = s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float)
        return val.fillna(999)

    sorted_df = df.sort_values(by="participant", key=sort_key).reset_index(drop=True)

    # Theme
    sns.set_theme(context="paper", style="ticks", 
                  rc={"xtick.direction": "in", "ytick.direction": "in"}, 
                  font_scale=FONT_SCALE)
    
    fig, ax = plt.subplots(figsize=FIG_SIZE)

    bar_edge_color = "black" if SHOW_ERROR_BARS else None
    bar_line_width = SPINES_WIDTH if SHOW_ERROR_BARS else 0

    # Bars - UPDATED y="mean_s"
    sns.barplot(
        data=sorted_df,
        x="participant",
        y="mean_s",
        color=BAR_COLOR,
        errorbar=None, 
        edgecolor=bar_edge_color,
        linewidth=bar_line_width,
        ax=ax
    )

    # Y-limit calculation - UPDATED columns
    if SHOW_ERROR_BARS:
        high_points = sorted_df["mean_s"] + sorted_df["std_s"].fillna(0)
    else:
        high_points = sorted_df["mean_s"]

    y_max = high_points.max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = 1.0

    if SHOW_ERROR_BARS:
        std_max = float(sorted_df["std_s"].fillna(0).max())
        top_padding = max(y_max * 0.02, std_max * 0.35)
    else:
        top_padding = y_max * 0.02

    ax.set_ylim(0, y_max + top_padding)

    # Error bars + labels
    for i in range(len(sorted_df)):
        mean = float(sorted_df.loc[i, "mean_s"])
        std = float(sorted_df.loc[i, "std_s"])

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
            y_text = mean / 2.0

            ax.text(
                i,
                y_text,
                f"{mean:.2f}",
                ha="center",
                va="center",
                color=VALUE_LABEL_COLOR,
                zorder=20,
                clip_on=True,
                fontsize="small",
            )

    # Dynamic Label based on metric
    label_map = {
        "e2e_blockchain": "Attestation Cycle Time (s)\nblockchain contribution",
        "total_lifecycle": "Attestation Cycle Time (s)",
        "evidence_tx_confirm": "Blockchain Write Time (s)",
        "evidence_call": "Local RPC Call Time (s)"
    }
    y_label = label_map.get(TARGET_METRIC, f"{TARGET_METRIC} (s)")

    ax.set_ylabel(y_label)
    ax.set_xlabel("")

    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()
    
    output_file = f"./barplot_attestation_cycle_time_by_robot_{mode}.pdf"
    out_path = script_dir / output_file
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)
    return True

def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path = (script_dir / INPUT_FILE).resolve()
    
    if not csv_path.exists():
        print(f"[ERR] CSV not found at: {csv_path}")
        sys.exit(1)

    df = pd.read_csv(csv_path)

    required = {"n_robots", "mode", "participant", "mean_s", "std_s", "metric", "role"}
    missing = required - set(df.columns)
    if missing:
        print(f"[ERR] CSV missing required columns: {missing}")
        print(f"      Found columns: {df.columns.tolist()}")
        sys.exit(1)

    # Generate plots for each mode
    for mode in MODES:
        # --- FILTERING ---
        mode_df = df[
            (df["n_robots"] == N_ROBOTS) &
            (df["mode"]     == mode) &
            (df["role"]     == TARGET_ROLE) &
            (df["metric"]   == TARGET_METRIC)
        ].copy()

        if not generate_plot(mode_df, mode, script_dir):
            print(f"[WARN] Failed to generate plot for mode '{mode}'")
            continue

if __name__ == "__main__":
    main()