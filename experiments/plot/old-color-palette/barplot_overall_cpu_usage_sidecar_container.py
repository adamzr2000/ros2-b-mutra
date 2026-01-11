#!/usr/bin/env python3

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import re

INPUT_FILE = "../data/docker-stats/_summary/overall_resource_usage_per_container.csv"
OUTPUT_FILE = "./overall_cpu_usage_sidecar_container.pdf"

FONT_SCALE = 1.5
LINE_WIDTH = 1.5
SPINES_WIDTH = 1.5
FIG_SIZE = (8, 5)

EDGE_COLORS = [
    "#8B4513",  # brown
    "#0000FF",  # blue
    "#FF0000",  # red
    "#008000",  # green
    "#FF7F00",  # orange
    "#00A6A6",  # teal
]


def to_rgb(hex_color: str):
    """Convert '#RRGGBB' to floats (r,g,b) in 0..1."""
    hex_color = hex_color.strip()
    if hex_color.startswith("#"):
        hex_color = hex_color[1:]
    if len(hex_color) != 6:
        raise ValueError(f"Expected 6-hex-digit color, got: {hex_color!r}")
    r = int(hex_color[0:2], 16) / 255.0
    g = int(hex_color[2:4], 16) / 255.0
    b = int(hex_color[4:6], 16) / 255.0
    return r, g, b


def _lighten(hex_color: str, factor: float = 0.65) -> str:
    """Lighten hex color towards white by 'factor' (0..1)."""
    r, g, b = to_rgb(hex_color)
    r = r + (1 - r) * factor
    g = g + (1 - g) * factor
    b = b + (1 - b) * factor
    return "#{:02X}{:02X}{:02X}".format(int(r * 255), int(g * 255), int(b * 255))


# Auto-generate face colors from edge colors
FACE_COLORS = [_lighten(c, factor=0.6) for c in EDGE_COLORS]


def _order_key(label: str):
    # SECaaS goes first
    if label == "SECaaS":
        return (0, 0)
    # RobotX goes next, sorted by X (numeric)
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (1, int(m.group(1)))
    # Anything else after, alpha
    return (2, label.lower())


def _clean_container_label(raw: str) -> str:
    """
    Turn 'robot1-sidecar' -> 'Robot1', 'secaas-sidecar' -> 'SECaaS',
    and keep a nice short label for plotting.
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

    # Expect columns: container, cpu_percent_mean, cpu_percent_std
    needed = {"container", "cpu_percent_mean", "cpu_percent_std"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(
            f"CSV is missing required columns: {missing}. "
            "Run summarize_resource_usage.py first."
        )

    # Clean labels
    df["Container"] = df["container"].apply(_clean_container_label)
    plot_df = df[["Container", "cpu_percent_mean", "cpu_percent_std"]].copy()

    # --- Enforce custom order: SECaaS, Robot1..N, then others alpha ---
    ordered_labels = sorted(plot_df["Container"].unique(), key=_order_key)
    plot_df["Container"] = pd.Categorical(
        plot_df["Container"], categories=ordered_labels, ordered=True
    )
    plot_df = plot_df.sort_values("Container").reset_index(drop=True)

    # Seaborn theme
    sns.set_theme(context="paper", style="ticks", font_scale=FONT_SCALE)

    # Plot
    fig, ax = plt.subplots(figsize=FIG_SIZE)

    # Draw bars with per-category colors via hue
    sns.barplot(
        data=plot_df,
        x="Container",
        y="cpu_percent_mean",
        hue="Container",
        palette=FACE_COLORS,
        linewidth=LINE_WIDTH,
        legend=False,
        errorbar=None,  # add std manually
        ax=ax,
    )

    # Set y-limit with headroom for labels above error bars
    y_max = (plot_df["cpu_percent_mean"] + plot_df["cpu_percent_std"].fillna(0)).max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = max(1.0, plot_df["cpu_percent_mean"].max() if len(plot_df) else 1.0)
    ax.set_ylim(0, y_max * 1.20)

    # Style edges, add error bars, add value labels above (mean + std)
    for i, bar in enumerate(ax.patches):
        current_edge = EDGE_COLORS[i % len(EDGE_COLORS)]
        bar.set_edgecolor(current_edge)

        mean = float(plot_df.loc[i, "cpu_percent_mean"])
        std = float(plot_df.loc[i, "cpu_percent_std"])

        # Error bars
        ax.errorbar(
            x=i,
            y=mean,
            yerr=std,
            fmt="none",
            ecolor=current_edge,
            elinewidth=LINE_WIDTH,
            capsize=4,
            capthick=1.5,
            zorder=10,
        )

        # Value label above the error bar
        x_center = bar.get_x() + bar.get_width() / 2.0
        y_text = mean + std + 0.015 * ax.get_ylim()[1]  # small offset
        ax.text(
            x_center,
            y_text,
            f"{mean:.2f}",
            ha="center",
            va="bottom",
            color=current_edge,
            fontsize=plt.rcParams["font.size"],
            zorder=20,
            clip_on=False,
        )

    ax.set_ylabel("CPU usage (%)\nwhere 100% ≈ one logical core")
    ax.set_xlabel("")
    ax.set_title("CPU usage of attestation-sidecar container")

    # Grid + spines
    ax.set_axisbelow(True)
    ax.grid(axis="y", linestyle="--", linewidth=1.0, alpha=0.8)

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
