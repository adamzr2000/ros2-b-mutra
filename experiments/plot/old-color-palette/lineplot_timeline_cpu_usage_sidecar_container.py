#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import re
from pathlib import Path

INPUT_FILE = "../data/docker-stats/_summary/timeline_resource_usage_per_container.csv"
# INPUT_FILE = "../data/docker-stats/_summary/timeline_resource_usage_per_container_run1.csv"
OUTPUT_FILE = "./timeline_cpu_usage_sidecar_container.pdf"
LINE_WIDTH = 1.8
FONT_SCALE = 1.5
SPINES_WIDTH = 1.5

# --- Smoothing (Option 1): time-based rolling mean ---
# Use a small window to reduce 1s sampling noise while preserving trends.
WINDOW_S = 5

# Colors to use in order; will cycle if there are more series than colors
COLORS = [
    "#8B4513",  # brown
    "#0000FF",  # blue
    "#FF0000",  # red
    "#008000",  # green
    "#FF7F00",  # orange
    "#00A6A6",  # teal
]


def _clean_container_label(raw: str) -> str:
    name = str(raw).replace("-sidecar", "").strip()
    low = name.lower()
    if low.startswith("secaas"):
        return "SECaaS"
    if low.startswith("robot"):
        return "Robot" + low.replace("robot", "")
    return name.title()

def _order_key(label: str):
    if label == "SECaaS":
        return (0, 0)
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (1, int(m.group(1)))
    return (2, label.lower())

def _apply_time_rolling_mean(df: pd.DataFrame, cpu_col: str) -> pd.DataFrame:
    """
    Apply a per-container, time-based rolling mean over WINDOW_S seconds.
    Uses t_rel_s as timebase; handles irregular sampling.
    """
    df = df.copy()
    df["t_td"] = pd.to_timedelta(df["t_rel_s"], unit="s")
    out = []
    for label, g in df.groupby("Container", sort=False, observed=True):
        g = g.sort_values("t_rel_s").set_index("t_td")
        g[cpu_col + "_sm"] = g[cpu_col].rolling(f"{WINDOW_S}s", min_periods=1).mean()
        out.append(g.reset_index())
    return pd.concat(out, ignore_index=True)

def main():
    csv_path = Path(INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # Choose cpu column name
    cpu_col = None
    for cand in ("cpu_percent", "cpu_percent_mean"):
        if cand in df.columns:
            cpu_col = cand
            break
    if cpu_col is None:
        raise SystemExit("Expected 'cpu_percent' or 'cpu_percent_mean' column in the timeline CSV.")

    # Required columns
    needed = {"container", "t_rel_s", cpu_col}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"Timeline CSV missing columns: {missing}")

    # Clean labels and enforce order
    df["Container"] = df["container"].apply(_clean_container_label)
    ordered_labels = sorted(df["Container"].unique(), key=_order_key)
    df["Container"] = pd.Categorical(df["Container"], categories=ordered_labels, ordered=True)

    # Smooth (time-based rolling mean)
    df = _apply_time_rolling_mean(df, cpu_col)
    y_col = cpu_col + "_sm"

    # Build a stable palette mapping Container -> color
    palette = {label: COLORS[i % len(COLORS)] for i, label in enumerate(ordered_labels)}

    # Theme
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "in", "ytick.direction": "in"}, font_scale=FONT_SCALE)


    # Plot (single axes, multiple lines)
    fig, ax = plt.subplots(figsize=(9, 5.5))
    sns.lineplot(
        data=df.sort_values(["Container", "t_rel_s"]),
        x="t_rel_s",
        y=y_col,
        hue="Container",
        hue_order=ordered_labels,
        palette=palette,
        ax=ax,
        linewidth=LINE_WIDTH,
        estimator=None,  # plotting already-smoothed values
        legend=True,
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CPU usage (%)")
    ax.set_title(f"CPU usage of attestation-sidecar container over time (rolling mean {WINDOW_S}s)")
    ax.grid(axis="both", linestyle="--", linewidth=1.0, alpha=0.8)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    # Legend
    leg = ax.legend(loc="upper right", frameon=True)
    # frame = leg.get_frame()
    # frame.set_edgecolor("black")
    # frame.set_linewidth(SPINES_WIDTH)

    ax.set_xlim(left=0)

    plt.tight_layout()
    # plt.show()

    # Save
    fig.savefig(OUTPUT_FILE, dpi=300)
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)

if __name__ == "__main__":
    main()
