#!/usr/bin/env python3
"""
Publication rate over time — continuous mode, both topics, single representative run.

Two panels (scan | odom), each showing:
  - Gray dashed   : No Attestation (baseline)
  - Purple solid  : Continuous Attestation

X-axis : elapsed time (s) from start of the run
Y-axis : mean per-robot publish rate (Hz) in BIN_S windows
"""

from pathlib import Path
import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
import seaborn as sns

RESULTS_DIR = Path(__file__).parent.parent / "data/performance-benchmark/results"

RUN        = "run1"   # which run to display
BIN_S      = 5.0      # time-bin width in seconds
ROLLING    = 1         # rolling-mean window over bins (1 = no smoothing)
FONT_SCALE = 1.4
LINE_WIDTH = 2.0

TOPICS = ["scan", "odom"]
TOPIC_DISPLAY = {"scan": "LiDAR (/scan)", "odom": "Odometry (/odom)"}

CONDITIONS = [
    ("no_sidecar",   "baseline",   "No Attestation",        "#555555", "--"),
    ("with_sidecar", "continuous", "Continuous Attestation", "#6B3FA0", "-"),
]


def rate_over_time(csv_path: Path, bin_s: float) -> pd.Series:
    """Return mean per-robot Hz per time bin (index = bin centre in seconds)."""
    df = pd.read_csv(csv_path)
    t0 = df["wall_stamp_ns"].min()
    df["t_s"] = (df["wall_stamp_ns"] - t0) / 1e9

    bins    = np.arange(0, df["t_s"].max() + bin_s, bin_s)
    centres = (bins[:-1] + bins[1:]) / 2

    robot_series = []
    for _, grp in df.groupby("robot"):
        counts, _ = np.histogram(grp["t_s"].values, bins=bins)
        robot_series.append(counts / bin_s)

    mat = np.array(robot_series)            # shape (n_robots, n_bins)
    mean_hz = mat.mean(axis=0)
    std_hz  = mat.std(axis=0)
    # drop first and last bins — partial windows skew the rate
    return pd.DataFrame({
        "mean": mean_hz[1:-1],
        "std":  std_hz[1:-1],
    }, index=centres[1:-1])


def main():
    script_dir = Path(__file__).parent.resolve()

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    fig, axes = plt.subplots(1, len(TOPICS), figsize=(5.5 * len(TOPICS), 4.2),
                             gridspec_kw={"wspace": 0.30})
    if len(TOPICS) == 1:
        axes = [axes]

    for ax, topic in zip(axes, TOPICS):
        for cond, mode, label, color, ls in CONDITIONS:
            csv_path = RESULTS_DIR / topic / cond / mode / f"{RUN}.csv"
            if not csv_path.exists():
                print(f"[WARN] Not found: {csv_path}")
                continue

            df_run = rate_over_time(csv_path, BIN_S)
            mean = df_run["mean"].rolling(ROLLING, center=True, min_periods=1).mean()
            std  = df_run["std"].rolling(ROLLING,  center=True, min_periods=1).mean()

            ax.plot(mean.index, mean.values,
                    color=color, linewidth=LINE_WIDTH, linestyle=ls,
                    label=label, zorder=3)
            ax.fill_between(mean.index,
                            mean - std, mean + std,
                            color=color, alpha=0.15, zorder=2)
            print(f"[{topic}/{cond}] {RUN}: "
                  f"min={mean.min():.2f}  max={mean.max():.2f}  "
                  f"mean={mean.mean():.2f} Hz")

        ax.set_title(TOPIC_DISPLAY.get(topic, topic), pad=7)
        ax.set_xlabel("Elapsed time (s)")
        ax.set_ylabel("Publish rate (Hz)")
        ax.set_xlim(left=0)
        ax.set_ylim(bottom=0)
        ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
        ax.grid(axis="both", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)
        ax.legend(loc="lower right", frameon=True, framealpha=0.9, fancybox=True,
                  fontsize=plt.rcParams.get("legend.fontsize", 9))

    out_path = script_dir / "lineplot_topic_rate_over_time.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
