#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns
import re
from pathlib import Path

INPUT_FILE_TEMPLATE = "../data/docker-stats/_summary/timeline_resource_usage_per_container_run{run}.csv"
TARGET_RUN = 1
N_ROBOTS = 4
MODES = ["startup", "continuous"]

LINE_WIDTH        = 2
FONT_SCALE        = 1.5
SPINES_WIDTH      = 1.0
THREE_PANEL_HEIGHT = 9.5
WINDOW_S          = 2      # rolling-mean smoothing window (seconds)


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
    csv_path   = (script_dir / INPUT_FILE_TEMPLATE.format(run=TARGET_RUN)).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    needed = {"n_robots", "mode", "container", "t_rel_s",
              "cpu_percent", "net_rx_mb", "net_tx_mb"}
    missing = needed - set(df.columns)
    if missing:
        raise SystemExit(f"Timeline CSV missing columns: {missing}. "
                         "Run summarize_docker_stats.py first.")

    for mode in MODES:
        subset = df[(df["n_robots"] == N_ROBOTS) & (df["mode"] == mode)].copy()
        if subset.empty:
            print(f"[WARN] No data for N={N_ROBOTS} mode={mode}, skipping.")
            continue
        generate_plot(subset, N_ROBOTS, mode, script_dir)


def generate_plot(df, n_robots, mode, script_dir):
    df = df.copy()
    df["Container"] = df["container"].apply(_clean_container_label)

    # CPU % → vCPUs
    df["cpu_vcpus"] = df["cpu_percent"] / 100.0

    # Cumulative net MB → throughput kbit/s (per container)
    for container, idx in df.groupby("Container").groups.items():
        g  = df.loc[idx].sort_values("t_rel_s")
        dt = g["t_rel_s"].diff()
        for src, dst in [("net_rx_mb", "net_rx_kbps"), ("net_tx_mb", "net_tx_kbps")]:
            rate = (df.loc[g.index, src].diff() / dt * 8000)
            df.loc[g.index, dst] = (
                rate.where(dt > 0, 0.0)
                    .replace([float("inf"), -float("inf")], 0.0)
                    .fillna(0.0).clip(lower=0.0)
            )

    containers = sorted(df["Container"].unique(), key=_label_order)

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    palette   = sns.color_palette("tab10", n_colors=len(containers))
    color_map = dict(zip(containers, palette))

    fig = plt.figure(figsize=(10, THREE_PANEL_HEIGHT), constrained_layout=True)
    gs  = fig.add_gridspec(3, 1, height_ratios=[1.0, 1.0, 1.0])
    ax_cpu = fig.add_subplot(gs[0, 0])
    ax_rx  = fig.add_subplot(gs[1, 0], sharex=ax_cpu)
    ax_tx  = fig.add_subplot(gs[2, 0], sharex=ax_cpu)

    panels = [
        (ax_cpu, "cpu_vcpus",    "CPU usage (vCPUs)"),
        (ax_rx,  "net_rx_kbps",  "Net RX (kbit/s)"),
        (ax_tx,  "net_tx_kbps",  "Net TX (kbit/s)"),
    ]

    for ax, col, ylabel in panels:
        y_max = 0.0
        for c in containers:
            sub   = df[df["Container"] == c].sort_values("t_rel_s").copy()
            if sub.empty:
                continue
            color = color_map[c]
            t     = sub["t_rel_s"].values
            sm    = _rolling(sub[col], sub["t_rel_s"])
            ax.plot(t, sm, color=color, linewidth=LINE_WIDTH, label=c)
            y_max = max(y_max, sm.max() if len(sm) else 0.0)

        ax.set_ylabel(ylabel)
        ax.set_axisbelow(True)
        ax.grid(axis="both", linestyle="-", linewidth=1.0, alpha=0.8)
        for side in ("top", "right", "bottom", "left"):
            ax.spines[side].set_color("black")
            ax.spines[side].set_linewidth(SPINES_WIDTH)
        y_max = y_max if not pd.isna(y_max) and y_max > 0 else 1.0
        ax.set_ylim(0, y_max * 1.08)

    ax_cpu.set_xlim(left=0)
    ax_cpu.tick_params(axis="x", labelbottom=False)
    ax_rx.tick_params(axis="x", labelbottom=False)
    ax_cpu.set_xlabel("")
    ax_rx.set_xlabel("")
    ax_tx.set_xlabel("Time (s)")
    ax_cpu.set_title(f"N={n_robots}  mode={mode}  rolling mean={WINDOW_S}s")

    handles, labels = ax_cpu.get_legend_handles_labels()
    ax_cpu.legend(handles, labels, loc="upper right", frameon=True, ncol=len(containers))

    out_path = script_dir / f"./timeplot_docker_stats_N{n_robots}_{mode}_run{TARGET_RUN}.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved plot to: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
