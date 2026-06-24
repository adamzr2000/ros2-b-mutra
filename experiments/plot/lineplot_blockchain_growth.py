#!/usr/bin/env python3
"""
Blockchain chain growth rate (KB/s) vs fleet size — one line per ITERQ value.

N ∈ {4, 8, 16, 32, 64}, ITERQ ∈ {1, 2, 4, 8}, SSP = 20 s.
Dashed gray line shows the idle (no-attestation) baseline.
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

SSP_MS    = 20000
ITERQ_VALS = [1, 2, 4, 8]

INPUT_FILE = "../data/blockchain-stats/_summary/blockchain_stats_summary.csv"
IDLE_FILE  = "../data/blockchain-stats/_summary/blockchain_stats_idle.csv"

FONT_SCALE = 1.45
LINEWIDTH  = 1.8
MARKERSIZE = 7
COLOR_IDLE = "gray"

_PALETTE = ["#b0a2d8", "#8e7dbb", "#6a5d99", "#3d3460"]  # light→dark: IterQ 1→2→4→8
STYLES = {
    1: {"color": _PALETTE[0], "marker": "o"},
    2: {"color": _PALETTE[1], "marker": "s"},
    4: {"color": _PALETTE[2], "marker": "^"},
    8: {"color": _PALETTE[3], "marker": "D"},
}

def main():
    script_dir = Path(__file__).parent.resolve()

    csv_path = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"Summary CSV not found: {csv_path}")

    idle_path = (script_dir / IDLE_FILE).resolve()
    if not idle_path.exists():
        raise SystemExit(f"Idle CSV not found: {idle_path}")

    df        = pd.read_csv(csv_path)
    idle_kbps = pd.read_csv(idle_path).iloc[0]["bytes_per_s"] / 1e3

    df = df[df["ssp_ms"] == SSP_MS].sort_values(["iterq", "N"])

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, ax = plt.subplots(figsize=(7, 3.4))
    plt.subplots_adjust(left=0.12, right=0.97, top=0.97, bottom=0.16)

    n_vals = sorted(df["N"].unique())

    print(f"{'IterQ':<8} {'N':<6} {'KB/s':>10}")
    print("-" * 26)
    for iterq in ITERQ_VALS:
        sub = df[df["iterq"] == iterq].sort_values("N")
        if sub.empty:
            continue
        x = sub["N"].to_numpy()
        y = sub["bytes_per_s"].to_numpy() / 1e3
        st = STYLES[iterq]
        ax.plot(x, y, color=st["color"], marker=st["marker"], markersize=MARKERSIZE,
                markeredgecolor="none",
                linewidth=LINEWIDTH, zorder=3, label=f"IterQ = {iterq}")
        for n, kbps in zip(x, y):
            print(f"{iterq:<8} {n:<6} {kbps:>10.4f}")

    print("-" * 26)
    print(f"{'Idle':<8} {'—':<6} {idle_kbps:>10.4f}")

    ax.axhline(idle_kbps, color=COLOR_IDLE, linewidth=1.4, linestyle="--",
               zorder=2, label="Idle")

    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("Blockchain growth rate (KB/s)")
    ax.set_xticks(n_vals)
    ax.set_xticklabels([str(n) for n in n_vals])
    ax.set_ylim(bottom=0)
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
    ax.grid(axis="both", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    ax.legend(loc="upper left", frameon=True, framealpha=0.9, fancybox=False,
              edgecolor="black", borderpad=0.4, handlelength=1.5, fontsize="small")

    out_path = script_dir / "lineplot_blockchain_growth.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
