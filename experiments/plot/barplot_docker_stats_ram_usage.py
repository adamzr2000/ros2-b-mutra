#!/usr/bin/env python3

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import re

# ---- Config ----
INPUT_FILE = "../data/docker-stats/_summary/overall_resource_usage_per_container.csv"
OUTPUT_FILE = "./overall_ram_usage_docker_stats.pdf"

FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (8, 5)

# Toggle features
SHOW_VALUE_LABELS = True
SHOW_ERROR_BARS = True

# Styling
ERR_COLOR = "black"

def _order_key(label: str):
    """
    Sorting logic:
    1. Robots (Robot1, Robot2...)
    2. Alphabetical for others
    3. SECaaS last
    """
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (0, int(m.group(1)))

    if label != "SECaaS":
        return (1, label.lower())

    return (2, 0)


def _clean_container_label(raw: str) -> str:
    """
    Turn 'robot1-sidecar' -> 'Robot1', 'secaas-sidecar' -> 'SECaaS'
    """
    name = raw.replace("-sidecar", "").strip()
    lower = name.lower()

    if lower.startswith("secaas"):
        return "SECaaS"

    if lower.startswith("robot"):
        return "Robot" + lower.replace("robot", "")

    return name.title()


def main():
    # Load summary
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    needed = {"container", "mem_mb_mean", "mem_mb_std"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(
            f"CSV is missing required columns: {missing}. "
            "Run summarize_resource_usage.py first."
        )

    # Clean labels
    df["Container"] = df["container"].apply(_clean_container_label)
    plot_df = df[["Container", "mem_mb_mean", "mem_mb_std"]].copy()

    # --- Enforce custom order: Robot1..N, others, SECaaS ---
    ordered_labels = sorted(plot_df["Container"].unique(), key=_order_key)
    plot_df["Container"] = pd.Categorical(
        plot_df["Container"], categories=ordered_labels, ordered=True
    )
    plot_df = plot_df.sort_values("Container").reset_index(drop=True)

    # Theme + palette + Inward Ticks
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)

    palette = sns.color_palette("colorblind", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Determine edge style based on flags
    # Scientific style (boxed) if error bars are on, otherwise flat.
    bar_edge_color = "black" if SHOW_ERROR_BARS else None
    bar_line_width = SPINES_WIDTH if SHOW_ERROR_BARS else 0

    # Plot Bars
    sns.barplot(
        data=plot_df,
        x="Container",
        y="mem_mb_mean",
        hue="Container",
        palette=color_map,
        legend=False,
        errorbar=None,  # add std manually
        edgecolor=bar_edge_color,
        linewidth=bar_line_width,
        ax=ax,
    )

    # Y-limit headroom logic
    if SHOW_ERROR_BARS:
        high_points = plot_df["mem_mb_mean"] + plot_df["mem_mb_std"].fillna(0)
    else:
        high_points = plot_df["mem_mb_mean"]

    y_max = high_points.max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = max(1.0, plot_df["mem_mb_mean"].max() if len(plot_df) else 1.0)
    
    ax.set_ylim(0, y_max * 1.20)

    # Error bars + Value labels
    for i in range(len(plot_df)):
        mean = float(plot_df.loc[i, "mem_mb_mean"])
        std = float(plot_df.loc[i, "mem_mb_std"])

        # 1. Draw Error Bars (if enabled)
        if SHOW_ERROR_BARS:
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

        # 2. Draw Value Labels (if enabled)
        if SHOW_VALUE_LABELS:
            # Position logic: If error bars exist, put label above whisker. Else above bar.
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

    # Formatting
    ax.set_ylabel(f"RAM usage (MB)\nattestation-sidecar container")
    ax.set_xlabel("")

    # Tighten X-axis
    ax.set_xlim(-0.5, len(plot_df) - 0.5)

    # Grid + spines
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="-", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    plt.tight_layout()

    # Save
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()