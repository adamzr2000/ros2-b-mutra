#!/usr/bin/env python3
"""
Total fleet attestation throughput vs fleet size (continuous mode).

X-axis : N (number of robots)
Y-axis : total attestation rounds per minute (all robots combined)
Reference line: perfect linear scaling  y = PER_ROBOT_RATE * N

A data line that sits on the linear reference proves the blockchain
coordination does not become a bottleneck as the fleet grows.
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import seaborn as sns

INPUT_FILE            = "../data/attestation-times/_summary/durations_raw.csv"
EXPERIMENT_DURATION_S = 120
ATTESTATION_INTERVAL_S = 10

FONT_SCALE  = 1.6
MARKER_SIZE = 8
LINE_WIDTH  = 2.0


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    required = {"n_robots", "mode", "role", "metric", "run"}
    missing  = required - set(df.columns)
    if missing:
        raise SystemExit(f"Missing columns: {missing}")

    # Total attestations per (n_robots, run) in continuous mode
    sub = df[
        (df["mode"]   == "continuous") &
        (df["role"]   == "prover") &
        (df["metric"] == "total_lifecycle")
    ]
    counts = (
        sub.groupby(["n_robots", "run"])
           .size()
           .reset_index(name="total_attestations")
    )
    duration_min = EXPERIMENT_DURATION_S / 60.0
    counts["fleet_rounds_per_min"] = counts["total_attestations"] / duration_min

    summary = (
        counts.groupby("n_robots")["fleet_rounds_per_min"]
              .agg(mean="mean", std="std")
              .reset_index()
    )
    summary["std"] = summary["std"].fillna(0.0)

    # Per-robot rate from N=4 (most stable, std=0) as the slope reference
    per_robot_rate = summary.loc[summary["n_robots"] == summary["n_robots"].min(),
                                 "mean"].iloc[0] / summary["n_robots"].min()

    print(f"\n{'N':>6}  {'fleet rounds/min':>18}  {'std':>8}")
    print("-" * 38)
    for _, row in summary.iterrows():
        print(f"{int(row['n_robots']):>6}  {row['mean']:>18.2f}  {row['std']:>8.4f}")
    print(f"\n  Per-robot rate (slope): {per_robot_rate:.4f} rounds/min/robot")
    print(f"  Linear reference: y = {per_robot_rate:.2f} × N\n")

    # ── Plot ──────────────────────────────────────────────────────────────────
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})
    c_main = "#336699"   # steel blue
    c_ref  = "gray"

    fig, ax = plt.subplots(figsize=(7, 4.0))

    n_vals = summary["n_robots"].values
    means  = summary["mean"].values
    stds   = summary["std"].values

    # Linear reference line across the full x range
    n_ref = np.linspace(n_vals[0] - 1, n_vals[-1] + 1, 200)
    ax.plot(n_ref, per_robot_rate * n_ref,
            color=c_ref, linewidth=1.5, linestyle="--", zorder=4,
            label="Ideal linear scaling")

    # Measured fleet throughput — markers only (no connecting line)
    ax.errorbar(n_vals, means, yerr=stds,
                fmt="o", color=c_main, capsize=4, markersize=MARKER_SIZE,
                linewidth=1.2, zorder=3,
                label="Measured fleet throughput")

    # Axes
    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("Attestation cycles / min")
    ax.set_xticks(n_vals)
    ax.set_xticklabels([str(n) for n in n_vals])
    ax.set_ylim(0, means.max() * 1.25)
    ax.set_xlim(n_vals[0] - 2, n_vals[-1] + 2)
    ax.grid(axis="both", linestyle="-", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    # Legend inside axes
    ax.legend(loc="upper left",
              ncol=1, frameon=True, framealpha=0.9, fancybox=True,
              fontsize=plt.rcParams.get("legend.fontsize", 9))

    # fig.suptitle("Fleet attestation throughput — Continuous mode",
    #              y=-0.04, fontsize=plt.rcParams.get("axes.titlesize", 11))

    plt.subplots_adjust(top=0.92)

    out_path = script_dir / "lineplot_attestation_throughput_fleet.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
