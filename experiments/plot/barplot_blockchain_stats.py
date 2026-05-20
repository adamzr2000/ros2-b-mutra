#!/usr/bin/env python3
"""
Blockchain stats vs fleet size — bar chart (1×4).

Four subplots:
  1. Block gas utilization (%)
  2. Blockchain footprint (KB)
  3. Blockchain growth rate (KB/s)
  4. Total transactions per run

Footprint and growth-rate panels include a dashed idle reference line.
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

SSP_MS     = 20000   # filter: sidecar sleep period (ms)
ITERQ      = 1       # filter: rolling-hash queue depth

INPUT_FILE = "../data/blockchain-stats/_summary/blockchain_stats_summary.csv"
IDLE_FILE  = "../data/blockchain-stats/_summary/blockchain_stats_idle.csv"

FONT_SCALE = 1.6
COLOR      = "#2E7D52"   # forest green
COLOR_IDLE = "gray"
BAR_WIDTH  = 0.55


def main():
    script_dir = Path(__file__).parent.resolve()

    csv_path = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"Summary CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)
    df = df[(df["ssp_ms"] == SSP_MS) & (df["iterq"] == ITERQ)]
    df = df.set_index("N").sort_index()

    idle_path = (script_dir / IDLE_FILE).resolve()
    if not idle_path.exists():
        raise SystemExit(f"Idle summary CSV not found: {idle_path}")
    idle = pd.read_csv(idle_path).iloc[0]
    idle_total_bytes = idle["total_bytes"]
    idle_bytes_per_s = idle["bytes_per_s"]

    n_vals = df.index.to_numpy()
    x      = np.arange(len(n_vals))

    metrics = [
        ("gas_used_pct", "Block gas utilization (%)",    1.0),
        ("total_bytes",  "Blockchain footprint (KB)",     1e-3),
        ("bytes_per_s",  "Blockchain growth rate (KB/s)", 1e-3),
        ("total_tx",     "Total transactions",             1.0),
    ]

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, axes = plt.subplots(1, 4, figsize=(18, 5))

    for ax, (key, ylabel, scale) in zip(axes, metrics):
        vals = df[key].to_numpy() * scale

        ax.bar(x, vals, width=BAR_WIDTH, color=COLOR, zorder=3)

        if key == "total_bytes":
            ax.axhline(idle_total_bytes * scale, color=COLOR_IDLE,
                       linewidth=1.5, linestyle="--", zorder=4, label="Idle")
            ax.legend(loc="upper left", frameon=True, framealpha=0.9, fancybox=True)
        if key == "bytes_per_s":
            ax.axhline(idle_bytes_per_s * scale, color=COLOR_IDLE,
                       linewidth=1.5, linestyle="--", zorder=4, label="Idle")
            ax.legend(loc="upper left", frameon=True, framealpha=0.9, fancybox=True)

        ax.set_xlabel("Number of robots (N)")
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels([str(n) for n in n_vals])
        ax.set_ylim(bottom=0)
        ax.grid(axis="y", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)

    fig.suptitle(
        f"Blockchain load vs. fleet size  (block period 2 s, SSP {SSP_MS} ms, ITERQ {ITERQ})",
        fontsize=plt.rcParams.get("axes.titlesize", 11),
        y=1.02,
    )
    plt.tight_layout(w_pad=2.0)

    out_path = script_dir / "barplot_blockchain_stats.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
