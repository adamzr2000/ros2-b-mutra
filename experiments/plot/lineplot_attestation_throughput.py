#!/usr/bin/env python3
"""
Per-robot attestation throughput vs fleet size (continuous mode).

X-axis : N (number of robots)
Y-axis : attestation rounds per minute per robot (mean ± std across runs)
Reference line: theoretical maximum = 60 / (ATTESTATION_INTERVAL_S)

A flat line as N grows proves linear scaling — adding robots does not
degrade per-robot attestation throughput.
"""

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.lines as mlines
import seaborn as sns

INPUT_FILE            = "../data/attestation-times/_summary/durations_raw.csv"
EXPERIMENT_DURATION_S = 120          # --duration 120 used for continuous runs
ATTESTATION_INTERVAL_S = 10          # ATTESTATION_INTERVAL_MS=10000 from .env

FONT_SCALE = 1.35
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

    # Count completed attestations per (n_robots, run) in continuous mode
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
    counts["rounds_per_min_per_robot"] = (
        counts["total_attestations"] / duration_min / counts["n_robots"]
    )

    summary = (
        counts.groupby("n_robots")["rounds_per_min_per_robot"]
              .agg(mean="mean", std="std")
              .reset_index()
    )
    summary["std"] = summary["std"].fillna(0.0)

    print("Per-robot throughput summary:")
    print(summary.to_string(index=False))

    # ── Plot ──────────────────────────────────────────────────────────────────
    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    palette = sns.color_palette("tab10")
    c_main  = palette[0]
    c_ref   = "gray"

    fig, ax = plt.subplots(figsize=(7, 4.0))

    n_vals = summary["n_robots"].values
    means  = summary["mean"].values
    stds   = summary["std"].values

    # Theoretical maximum reference line
    t_max = 60.0 / ATTESTATION_INTERVAL_S

    # Print plotted values
    print(f"\n{'N':>6}  {'mean (rounds/min/robot)':>24}  {'std':>8}")
    print("-" * 44)
    for n, m, s in zip(n_vals, means, stds):
        print(f"{n:>6}  {m:>24.4f}  {s:>8.4f}")
    print(f"\n  Theoretical max: {t_max:.4f} rounds/min/robot")
    print(f"  Achieved:        {means.mean():.4f} rounds/min/robot  "
          f"({means.mean()/t_max*100:.1f}% of theoretical max)\n")

    # Data line
    ax.plot(n_vals, means, color=c_main, linewidth=LINE_WIDTH,
            marker="o", markersize=MARKER_SIZE, zorder=3,
            label="Measured (mean ± std)")
    ax.errorbar(n_vals, means, yerr=stds,
                fmt="none", color=c_main, capsize=4,
                linewidth=1.2, zorder=4)
    ax.axhline(t_max, color=c_ref, linewidth=1.2, linestyle="--", zorder=2,
               label=f"Theoretical max ({t_max:.0f} rounds/min, $T_{{attest}}$={ATTESTATION_INTERVAL_S}s)")

    # Axes
    ax.set_xlabel("Number of robots (N)")
    ax.set_ylabel("Attestation rounds / min / robot")
    ax.set_xticks(n_vals)
    ax.set_xticklabels([str(n) for n in n_vals])

    y_top = t_max * 1.20
    y_bot = means.min() * 0.85
    ax.set_ylim(y_bot, y_top)
    ax.set_xlim(n_vals[0] - 2, n_vals[-1] + 2)

    ax.grid(axis="y", linestyle="--", linewidth=0.7, alpha=0.75)
    ax.set_axisbelow(True)

    # Legend top-centre
    fig.legend(loc="upper center", bbox_to_anchor=(0.5, 1.01),
               ncol=2, frameon=True, framealpha=0.9, fancybox=True,
               fontsize=plt.rcParams.get("legend.fontsize", 9))

    # fig.suptitle("Per-robot attestation throughput — Continuous mode",
    #              y=-0.04, fontsize=plt.rcParams.get("axes.titlesize", 11))

    plt.subplots_adjust(top=0.82)

    out_path = script_dir / "lineplot_attestation_throughput.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
