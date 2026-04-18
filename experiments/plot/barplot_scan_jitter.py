#!/usr/bin/env python3
"""
Bar chart comparing /scan topic publish rate and jitter across conditions.

Three conditions: no_sidecar/baseline, with_sidecar/startup, with_sidecar/continuous.
Two panels: left = mean Hz, right = mean jitter (ms).
Error bars = std across runs.
"""

from pathlib import Path
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import seaborn as sns

INPUT_FILE = "../data/performance-benchmark/summary_topic_stats.csv"

FONT_SCALE    = 1.4
BAR_WIDTH     = 0.55
HEADROOM      = 1.15

# Display labels and order
CONDITION_LABELS = {
    ("no_sidecar",   "baseline"):   "No\nAttestation",
    ("with_sidecar", "startup"):    "Startup\nAttestation",
    ("with_sidecar", "continuous"): "Continuous\nAttestation",
}


def main():
    script_dir = Path(__file__).parent.resolve()
    csv_path   = (script_dir / INPUT_FILE).resolve()
    if not csv_path.exists():
        raise SystemExit(f"CSV not found: {csv_path}")

    df = pd.read_csv(csv_path)

    # Build ordered rows
    rows = []
    for (cond, mode), label in CONDITION_LABELS.items():
        sub = df[(df["condition"] == cond) & (df["mode"] == mode)]
        if sub.empty:
            print(f"[WARN] No data for {cond}/{mode}, skipping.")
            continue
        rows.append({
            "label":          label,
            "hz_mean":        float(sub["hz_mean"].iloc[0]),
            "hz_std":         float(sub["hz_std"].iloc[0]),
            "jitter_mean_ms": float(sub["jitter_mean_ms"].iloc[0]),
            "jitter_std_ms":  float(sub["jitter_std_ms"].iloc[0]),
        })

    if not rows:
        raise SystemExit("No data rows found.")

    sns.set_theme(context="paper", style="ticks",
                  rc={"xtick.direction": "out", "ytick.direction": "out"},
                  font_scale=FONT_SCALE)
    plt.rcParams.update({"font.family": "serif"})

    colors = ["#555555", "#336699", "#993333"]   # gray, steel blue, dark red
    xs       = list(range(len(rows)))
    labels   = [r["label"] for r in rows]

    fig, (ax_hz, ax_jit) = plt.subplots(
        1, 2, figsize=(10, 4.4),
        gridspec_kw={"wspace": 0.28},
    )

    for ax, metric, std_key, ylabel, title in [
        (ax_hz,  "hz_mean",        "hz_std",         "LiDAR publish rate (Hz)",   "Mean publish rate"),
        (ax_jit, "jitter_mean_ms", "jitter_std_ms",  "LiDAR jitter (ms)",         "Inter-arrival jitter"),
    ]:
        for i, (row, color) in enumerate(zip(rows, colors)):
            mean = row[metric]
            std  = row[std_key]
            ax.bar(i, mean, width=BAR_WIDTH, color=color,
                   edgecolor="black", linewidth=0.8, zorder=3)
            if std > 0:
                ax.errorbar(i, mean, yerr=std,
                            fmt="none", color="black",
                            capsize=4, linewidth=1.2, zorder=5)
            print(f"[VAL] {title}: {row['label'].replace(chr(10), ' '):<30} "
                  f"mean={mean:.3f}  std={std:.4f}")

        ax.set_xticks(xs)
        ax.set_xticklabels(labels, ha="center")
        ax.set_xlim(-0.6, len(rows) - 0.4)
        ax.set_ylim(0, max(row[metric] + row[std_key] for row in rows) * HEADROOM)
        ax.set_ylabel(ylabel)
        ax.tick_params(axis="both", which="major", length=6, width=1.0, direction="out")
        ax.grid(axis="y", which="major", linestyle="-", linewidth=0.7, alpha=0.75)
        ax.set_axisbelow(True)

    # Reference line at 5 Hz (nominal scan rate)
    ax_hz.axhline(5.0, color="gray", linewidth=1.0, linestyle="--", zorder=2)
    ax_hz.text(len(rows) - 0.42, 5.02, "5 Hz nominal",
               color="gray", fontsize=plt.rcParams.get("font.size", 9) * 0.82,
               va="bottom", ha="right")

    plt.subplots_adjust(top=0.92)

    out_path = script_dir / "barplot_scan_jitter.pdf"
    fig.savefig(out_path, dpi=300, bbox_inches="tight")
    print(f"\n[OK] Saved: {out_path}")
    plt.close(fig)


if __name__ == "__main__":
    main()
