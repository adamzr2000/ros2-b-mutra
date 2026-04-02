#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import re
from pathlib import Path

# INPUT_FILE = "../data/docker-stats/_summary/timeline_resource_usage_per_container.csv"
INPUT_FILE = "../data/docker-stats/_summary/timeline_resource_usage_per_container_run1.csv"
OUTPUT_FILE = "./timeplot_docker_stats.pdf"

LINE_WIDTH = 2
FONT_SCALE = 1.5
SPINES_WIDTH = 1.0
FIG_SIZE = (9, 5.5)

# --- Smoothing: time-based rolling mean ---
WINDOW_S = 5


def _auto_figsize(df: pd.DataFrame, x_col: str = "t_rel_s", group_col: str = "Container"):
    n_groups = int(df[group_col].nunique()) if group_col in df.columns else 1
    if x_col in df.columns and len(df):
        x_span = float(df[x_col].max() - df[x_col].min())
    else:
        x_span = 0.0

    width = 5.8 + min(x_span / 120.0, 3.0) + min(n_groups * 0.18, 2.0)
    height = 3.4 + min(n_groups * 0.22, 2.8)

    width = max(7.0, min(width, 12.5))
    height = max(4.0, min(height, 7.5))
    return (width, height)


def _clean_container_label(raw: str) -> str:
    name = str(raw).replace("-sidecar", "").strip()
    low = name.lower()
    if low.startswith("secaas"):
        return "SECaaS"
    if low.startswith("robot"):
        return "Robot" + low.replace("robot", "")
    return name.title()


def _order_key(label: str):
    # Robots first (Robot1..N)
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (0, int(m.group(1)))

    # Anything else next (alphabetical)
    if label != "SECaaS":
        return (1, label.lower())

    # SECaaS goes last
    return (2, 0)


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
    cpu_col_raw = None
    for cand in ("cpu_percent", "cpu_percent_mean"):
        if cand in df.columns:
            cpu_col_raw = cand
            break
    if cpu_col_raw is None:
        raise SystemExit("Expected 'cpu_percent' or 'cpu_percent_mean' column in the timeline CSV.")

    # Required columns
    needed = {"container", "t_rel_s", cpu_col_raw}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"Timeline CSV missing columns: {missing}")
    
    # --- CONVERT TO vCPUs ---
    # 100% usage = 1 vCPU
    cpu_vcpus_col = "cpu_vcpus"
    df[cpu_vcpus_col] = df[cpu_col_raw] / 100.0

    # Clean labels and enforce order
    df["Container"] = df["container"].apply(_clean_container_label)
    ordered_labels = sorted(df["Container"].unique(), key=_order_key)
    df["Container"] = pd.Categorical(df["Container"], categories=ordered_labels, ordered=True)

    # Smooth (time-based rolling mean)
    # Note: We now smooth the vCPU column
    df = _apply_time_rolling_mean(df, cpu_vcpus_col)
    y_col = cpu_vcpus_col + "_sm"

    # Theme + palette
    sns.set_theme(context="paper", style="ticks", rc={"xtick.direction": "out", "ytick.direction": "out"}, font_scale=FONT_SCALE)

    palette = sns.color_palette("tab10", n_colors=len(ordered_labels))
    color_map = dict(zip(ordered_labels, palette))

    # Plot (single axes, multiple lines)
    fig_size = _auto_figsize(df, x_col="t_rel_s", group_col="Container")
    fig, ax = plt.subplots(figsize=fig_size)
    sns.lineplot(
        data=df.sort_values(["Container", "t_rel_s"]),
        x="t_rel_s",
        y=y_col,
        hue="Container",
        hue_order=ordered_labels,
        palette=color_map,
        ax=ax,
        linewidth=LINE_WIDTH,
        estimator=None,  # plotting already-smoothed values
        legend=True,
    )

    ax.set_xlabel("Time (s)")
    ax.set_ylabel("CPU usage (vCPUs)")
    ax.set_title(f"rolling mean = {WINDOW_S}s")

    ax.set_axisbelow(True)
    ax.grid(axis="both", linestyle="-", linewidth=1.0, alpha=0.8)
    ax.set_xlim(left=0)

    # Y-axis auto top-padding (consistent with other plots)
    y_max = df[y_col].max()
    if pd.isna(y_max) or y_max <= 0:
        y_max = 1.0
    top_padding = y_max * 0.02
    ax.set_ylim(0, y_max + top_padding)

    for side in ("top", "right", "bottom", "left"):
        ax.spines[side].set_color("black")
        ax.spines[side].set_linewidth(SPINES_WIDTH)

    # Legend
    ax.legend(loc="upper right", frameon=True)

    plt.tight_layout()

    # Save
    fig.savefig(OUTPUT_FILE, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {OUTPUT_FILE}")
    plt.close(fig)


if __name__ == "__main__":
    main()