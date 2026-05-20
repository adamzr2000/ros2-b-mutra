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

FONT_SCALE = 1.8
LINEWIDTH  = 1.8
MARKERSIZE = 7
COLOR_IDLE = "gray"

# viridis sequential: low ITERQ (dark) → high ITERQ (light)
# wide spacing to maximise contrast between the 4 lines
_viridis = plt.cm.viridis([0.10, 0.38, 0.65, 0.88])
STYLES = {
    1: {"color": _viridis[0], "marker": "o"},
    2: {"color": _viridis[1], "marker": "s"},
    4: {"color": _viridis[2], "marker": "^"},
    8: {"color": _viridis[3], "marker": "D"},
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
                  rc={"xtick.direction": "in", "ytick.direction": "in"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, ax = plt.subplots(figsize=(7, 4.5))
    plt.subplots_adjust(left=0.12, right=0.97, top=0.95, bottom=0.14)

    n_vals = sorted(df["N"].unique())

    for iterq in ITERQ_VALS:
        sub = df[df["iterq"] == iterq].sort_values("N")
        if sub.empty:
            continue
        x = sub["N"].to_numpy()
        y = sub["bytes_per_s"].to_numpy() / 1e3
        st = STYLES[iterq]
        ax.plot(x, y, color=st["color"], marker=st["marker"], markersize=MARKERSIZE,
                markeredgecolor="black", markeredgewidth=0.8,
                linewidth=LINEWIDTH, zorder=3, label=f"ITERQ = {iterq}")

    ax.axhline(idle_kbps, color=COLOR_IDLE, linewidth=1.4, linestyle="--",
               zorder=2, label="Idle")

    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("Blockchain growth rate (KB/s)")
    ax.set_xticks(n_vals)
    ax.set_xticklabels([str(n) for n in n_vals])
    ax.set_ylim(bottom=0)
    ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="in")
    ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    ax.legend(loc="upper left", frameon=True, framealpha=0.9, fancybox=False,
              edgecolor="black", borderpad=0.4, handlelength=1.5)

    out_path = script_dir / "lineplot_blockchain_growth.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
