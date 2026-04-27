#!/usr/bin/env python3
"""
Summarize topic publish rate across conditions.

Per-run median Hz is computed for each run (median over all robots in that run),
then mean ± std across runs is reported. This matches the boxplot representation.

Expected layout:
  results/{topic}/{condition}/{mode}/run{N}.csv
  columns: robot, topic, ros_stamp_ns, wall_stamp_ns
"""

from pathlib import Path
import numpy as np
import pandas as pd

RESULTS_DIR = Path(__file__).parent / "results"

CONDITION_MODES = {
    "no_sidecar":   ["baseline"],
    "with_sidecar": ["continuous"],
}


def load_run_medians(run_dir: Path) -> np.ndarray:
    """Return one median-Hz value per run file (median over all robots in that run)."""
    medians = []
    for csv_path in sorted(run_dir.glob("run*.csv")):
        df = pd.read_csv(csv_path)
        intervals = []
        for _, grp in df.groupby("robot"):
            grp = grp.sort_values("wall_stamp_ns")
            intervals.append(grp["wall_stamp_ns"].diff().dropna().values / 1e6)
        if intervals:
            all_ivs = np.concatenate(intervals)
            medians.append(1000.0 / np.median(all_ivs))
    return np.array(medians)


def main():
    if not RESULTS_DIR.exists():
        print("[WARN] results/ directory not found. Run the benchmark first.")
        return

    topic_dirs = sorted(p for p in RESULTS_DIR.iterdir() if p.is_dir())
    if not topic_dirs:
        print("[WARN] No topic directories found under results/.")
        return

    summary_rows = []

    for topic_dir in topic_dirs:
        topic = topic_dir.name
        for condition, modes in CONDITION_MODES.items():
            for mode in modes:
                run_dir = topic_dir / condition / mode
                if not run_dir.exists():
                    continue

                vals = load_run_medians(run_dir)
                if vals.size == 0:
                    print(f"[WARN] No data for {topic}/{condition}/{mode}")
                    continue

                row = {
                    "topic":      topic,
                    "condition":  condition,
                    "mode":       mode,
                    "n_runs":     vals.size,
                    "hz_mean":    vals.mean(),
                    "hz_std":     vals.std(),
                    "hz_min":     vals.min(),
                    "hz_max":     vals.max(),
                    "hz_median":  np.median(vals),
                    "hz_p25":     np.percentile(vals, 25),
                    "hz_p75":     np.percentile(vals, 75),
                }
                for i, v in enumerate(vals, start=1):
                    row[f"hz_run{i}"] = v
                summary_rows.append(row)

    if not summary_rows:
        print("[WARN] No data found. Run the benchmark first.")
        return

    df = pd.DataFrame(summary_rows)

    print("\n=== Per-run median publish rate (mean ± std across runs) ===\n")
    print(f"{'topic':<10} {'condition':<15} {'mode':<12} {'runs':>5}  "
          f"{'hz_mean':>10}  {'hz_std':>8}  {'hz_min':>8}  {'hz_max':>8}")
    print("-" * 80)
    for _, r in df.iterrows():
        print(f"{r['topic']:<10} {r['condition']:<15} {r['mode']:<12} {int(r['n_runs']):>5}  "
              f"{r['hz_mean']:>10.4f}  {r['hz_std']:>8.4f}  "
              f"{r['hz_min']:>8.4f}  {r['hz_max']:>8.4f}")

    out = RESULTS_DIR.parent / "summary_topic_stats.csv"
    df.to_csv(out, index=False)
    print(f"\n[OK] Summary saved to {out}")


if __name__ == "__main__":
    main()
