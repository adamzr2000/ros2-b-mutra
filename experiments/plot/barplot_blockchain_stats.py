#!/usr/bin/env python3
"""
Blockchain stats vs fleet size — bar chart (1×3).

Three subplots:
  1. Block gas utilization (%)
  2. Blockchain footprint (KB)
  3. Total transactions per run

Bars show mean ± std across runs. Footprint panel includes a dashed
idle reference line.
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

INPUT_FILE = "../data/blockchain-stats/_summary/blockchain_stats_summary.csv"
IDLE_CSV   = "../data/blockchain-stats/results/idle/blockchain-idle-120s.csv"

FONT_SCALE  = 1.6
COLOR       = "#336699"
COLOR_IDLE  = "gray"
BAR_WIDTH   = 0.55
SHOW_ERRORS = False


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"Summary CSV not found: {csv_path}")

    df = pd.read_csv(csv_path, index_col="N")

    idle_path = (script_dir / IDLE_CSV).resolve()
    if not idle_path.exists():
        raise SystemExit(f"Idle CSV not found: {idle_path}")
    idle_total_bytes = pd.read_csv(idle_path)["size_bytes"].sum()

    n_vals = df.index.to_numpy()
    x      = np.arange(len(n_vals))

    metrics = [
        ("gas_used_pct", "Block gas utilization (%)", 1.0),
        ("total_bytes",  "Blockchain footprint (KB)",  1e-3),
        ("total_tx",     "Total transactions",          1.0),
    ]

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, axes = plt.subplots(1, 3, figsize=(14, 5))

    for ax, (key, ylabel, scale) in zip(axes, metrics):
        means = df[f"{key}_mean"].to_numpy() * scale
        stds  = df[f"{key}_std"].to_numpy()  * scale

        yerr = stds if SHOW_ERRORS else None
        ax.bar(x, means, width=BAR_WIDTH, color=COLOR,
               yerr=yerr, capsize=4,
               error_kw={"linewidth": 1.2, "zorder": 5},
               zorder=3)

        if key == "total_bytes":
            ax.axhline(idle_total_bytes * scale, color=COLOR_IDLE,
                       linewidth=1.5, linestyle="--", zorder=4, label="Idle")
            ax.legend(loc="upper left", frameon=True, framealpha=0.9,
                      fancybox=True)

        ax.set_xlabel("Number of robots (N)")
        ax.set_ylabel(ylabel)
        ax.set_xticks(x)
        ax.set_xticklabels([str(n) for n in n_vals])
        ax.set_ylim(bottom=0)
        ax.grid(axis="y", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)

    fig.suptitle(
        "Blockchain load vs. fleet size  (window 120 s, block period 2 s, attestation interval 20 s)",
        fontsize=plt.rcParams.get("axes.titlesize", 11),
        y=1.02,
    )
    plt.tight_layout(w_pad=3.0)

    out_path = script_dir / "barplot_blockchain_stats.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
