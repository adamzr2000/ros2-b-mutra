#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import matplotlib.patches as mpatches
import seaborn as sns
import re
from pathlib import Path
from functools import reduce

INPUT_STARTUP_TEMPLATE    = "../data/docker-stats/_summary/timeline_resource_usage_startup_run{run}.csv"
INPUT_CONTINUOUS_TEMPLATE = "../data/docker-stats/_summary/timeline_resource_usage_{variant}_run{run}.csv"
TARGET_RUN = 1
N_ROBOTS   = 4
MODES      = ["continuous"]
VARIANT    = "rr"   # ← continuous-mode variant to plot

LINE_WIDTH   = 2.6
FONT_SCALE   = 2.0
WINDOW_S     = 1      # rolling-mean smoothing window (seconds)
BAND_ALPHA   = 0.25   # opacity of ±std shaded band


def _clean_container_label(raw: str) -> str:
    name = str(raw).replace("-sidecar", "").strip()
    low  = name.lower()
    if low.startswith("secaas"):
        return "SECaaS"
    if low.startswith("robot"):
        return "Robot" + low.replace("robot", "").title()
    return name.title()


def _label_order(label: str):
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (0, int(m.group(1)))
    return (1, label)


def _rolling(series: pd.Series, t_rel: pd.Series) -> pd.Series:
    """Time-based rolling mean over WINDOW_S seconds."""
    df_tmp = pd.DataFrame({"val": series.values, "t": t_rel.values})
    df_tmp = df_tmp.sort_values("t").copy()
    df_tmp["t_td"] = pd.to_timedelta(df_tmp["t"], unit="s")
    df_tmp = df_tmp.set_index("t_td")
    df_tmp["sm"] = df_tmp["val"].rolling(f"{WINDOW_S}s", min_periods=1).mean()
    return df_tmp["sm"].values


def main():
    script_dir = Path(__file__).parent.resolve()

    needed = {"n_robots", "mode", "container", "t_rel_s",
              "cpu_percent", "net_tx_mb"}

    for mode in MODES:
        tmpl = INPUT_STARTUP_TEMPLATE if mode == "startup" else INPUT_CONTINUOUS_TEMPLATE
        csv_path = (script_dir / tmpl.format(run=TARGET_RUN, variant=VARIANT)).resolve()
        if not csv_path.exists():
            print(f"[WARN] CSV not found for mode={mode}: {csv_path}")
            continue

        df = pd.read_csv(csv_path)
        missing = needed - set(df.columns)
        if missing:
            print(f"[WARN] Timeline CSV missing columns: {missing}. Run summarize_docker_stats.py first.")
            continue

        subset = df[df["n_robots"] == N_ROBOTS].copy()
        if subset.empty:
            print(f"[WARN] No data for N={N_ROBOTS} mode={mode}, skipping.")
            continue
        generate_plot(subset, N_ROBOTS, mode, script_dir)


def generate_plot(df, n_robots, mode, script_dir):
    df = df.copy()
    df["Container"] = df["container"].apply(_clean_container_label)

    # CPU % → vCPUs
    df["cpu_vcpus"] = df["cpu_percent"] / 100.0

    # Cumulative net TX MB → throughput kbit/s (per container)
    for container, idx in df.groupby("Container").groups.items():
        g  = df.loc[idx].sort_values("t_rel_s")
        dt = g["t_rel_s"].diff()
        rate = (df.loc[g.index, "net_tx_mb"].diff() / dt * 8000)
        df.loc[g.index, "net_tx_kbps"] = (
            rate.where(dt > 0, 0.0)
                .replace([float("inf"), -float("inf")], 0.0)
                .fillna(0.0).clip(lower=0.0)
        )

    containers       = sorted(df["Container"].unique(), key=_label_order)
    robot_containers = [c for c in containers if c.startswith("Robot")]
    has_secaas       = "SECaaS" in containers

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})
    color_robot  = "#993333"
    color_secaas = "#336699"

    fig = plt.figure(figsize=(10, 5.5))
    gs  = fig.add_gridspec(2, 1, height_ratios=[1.0, 1.0], hspace=0.12,
                           left=0.10, right=0.97, top=0.87, bottom=0.12)
    ax_cpu = fig.add_subplot(gs[0])
    ax_tx  = fig.add_subplot(gs[1], sharex=ax_cpu)

    panels = [
        (ax_cpu, "cpu_vcpus",   "CPU (vCPUs)"),
        (ax_tx,  "net_tx_kbps", "Net TX (kbit/s)"),
    ]

    def _robot_band(col):
        """Return (t, mean, std) arrays aggregated across robot containers."""
        smoothed = {}
        for c in robot_containers:
            sub = df[df["Container"] == c].sort_values("t_rel_s").copy()
            sm  = _rolling(sub[col], sub["t_rel_s"])
            smoothed[c] = pd.Series(sm, index=sub["t_rel_s"].values)
        band = (
            pd.DataFrame(smoothed)
              .sort_index()
              .interpolate(method="index", limit_direction="both")
        )
        t    = band.index.values
        mean = band.mean(axis=1).values
        std  = band.std(axis=1).fillna(0.0).values
        return t, mean, std

    for ax, col, ylabel in panels:
        y_max = 0.0

        # ── Robot band ────────────────────────────────────────────────────────
        if robot_containers:
            t, mean, std = _robot_band(col)
            ax.plot(t, mean, color=color_robot, linewidth=LINE_WIDTH, zorder=3)
            ax.fill_between(t, (mean - std).clip(0), mean + std,
                            color=color_robot, alpha=BAND_ALPHA, zorder=2)
            y_max = max(y_max, (mean + std).max())

        # ── SECaaS line ───────────────────────────────────────────────────────
        if has_secaas:
            sub = df[df["Container"] == "SECaaS"].sort_values("t_rel_s").copy()
            sm  = _rolling(sub[col], sub["t_rel_s"])
            ax.plot(sub["t_rel_s"].values, sm,
                    color=color_secaas, linewidth=LINE_WIDTH, zorder=3)
            y_max = max(y_max, sm.max() if len(sm) else 0.0)

        ax.set_ylabel(ylabel)
        ax.set_axisbelow(True)
        ax.grid(axis="both", linestyle="-", linewidth=0.7, alpha=0.75)
        y_max = y_max if not pd.isna(y_max) and y_max > 0 else 1.0
        ax.set_ylim(0, y_max * 1.10)

    ax_cpu.set_xlim(left=0)
    ax_cpu.tick_params(axis="x", labelbottom=False)
    ax_tx.set_xlabel("Time (s)")

    # ── Figure legend (top centre) ────────────────────────────────────────────
    legend_handles = []
    if robot_containers:
        legend_handles += [
            mlines.Line2D([], [], color=color_robot, linewidth=LINE_WIDTH,
                          label="Robot sidecar"),
        ]
    if has_secaas:
        legend_handles.append(
            mlines.Line2D([], [], color=color_secaas, linewidth=LINE_WIDTH,
                          label="SECaaS")
        )

    fig.legend(handles=legend_handles,
               loc="upper center", bbox_to_anchor=(0.5, 1.0),
               ncol=len(legend_handles), framealpha=0.9,
               fancybox=False, edgecolor="black",
               borderpad=0.5, handlelength=1.4, columnspacing=1.2)
    
    out_path = script_dir / f"timeplot_docker_stats_N{n_robots}_{mode}_run{TARGET_RUN}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
