#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
from pathlib import Path

# ---- Config ----
INPUT_FILE = "../data/attestation-times/_summary/attestation_duration_per_robot.csv"
OUTPUT_FILE = "./barplot_attestation_cycle_time_per_robot.pdf"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.5
FIG_SIZE = (7, 4.5)

# Error bars + value labels
# ERR_COLOR = "0.2"
ERR_COLOR = "black"

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    required = {"robot", "mean_of_means_s", "std_of_means_s"}
    missing = required - set(df.columns)
    if missing:
        raise SystemExit(f"CSV missing required columns: {missing}")

    # Sort Robot1, Robot2, ...
    df = df.sort_values(
        "robot",
        key=lambda s: s.astype(str).str.extract(r"(\d+)").iloc[:, 0].astype(float).fillna(0)
    ).reset_index(drop=True)

    # Theme + palette
    sns.set_theme(context="paper", style="ticks", font_scale=FONT_SCALE)
    ordered_labels = df["robot"].astype(str).tolist()
    palette = sns.color_palette("tab10", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Bars
    sns.barplot(
        data=df,
        x="robot",
        y="mean_of_means_s",
        hue="robot",
        palette=color_map,
        legend=False,
        errorbar=None,  # add std manually
        ax=ax,
    )

    # Y-limit headroom
    y_max = (df["mean_of_means_s"] + df["std_of_means_s"].fillna(0)).max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = max(1.0, df["mean_of_means_s"].max() if len(df) else 1.0)
    ax.set_ylim(0, y_max * 1.20)

    # Error bars + labels
    for i in range(len(df)):
        mean = float(df.loc[i, "mean_of_means_s"])
        std = float(df.loc[i, "std_of_means_s"])

        ax.errorbar(
            x=i,
            y=mean,
            yerr=std,
            fmt="none",
            ecolor=ERR_COLOR,
            elinewidth=1.5,
            capsize=4,
            capthick=1.5,
            zorder=10,
        )

        y_text = mean + std + 0.015 * ax.get_ylim()[1]
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
