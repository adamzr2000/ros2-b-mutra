#!/usr/bin/env python3
"""
Summarize /scan topic inter-arrival jitter across conditions and modes.

Computes per-robot inter-arrival time statistics from the collected CSVs
and compares: no_sidecar vs with_sidecar  ×  startup vs continuous.

Expected layout:
  results/{condition}/{mode}/run{N}.csv
  columns: robot, ros_stamp_ns, wall_stamp_ns
"""

from pathlib import Path
import pandas as pd
import numpy as np

RESULTS_DIR = Path(__file__).parent / "results"

# no_sidecar has no attestation cycle → single "baseline" mode
# with_sidecar distinguishes startup vs continuous
CONDITION_MODES = {
    "no_sidecar":   ["baseline"],
    "with_sidecar": ["startup", "continuous"],
}


def process_run(csv_path: Path) -> pd.DataFrame:
    """Return per-robot inter-arrival stats for a single run CSV."""
    df = pd.read_csv(csv_path)
    rows = []
    for robot, grp in df.groupby("robot"):
        grp = grp.sort_values("wall_stamp_ns").reset_index(drop=True)
        if len(grp) < 2:
            continue
        intervals_ms = grp["wall_stamp_ns"].diff().dropna() / 1e6   # ns → ms
        rows.append({
            "robot":              robot,
            "n_msgs":             len(grp),
            "mean_interval_ms":   intervals_ms.mean(),
            "std_interval_ms":    intervals_ms.std(),
            "mean_hz":            1000.0 / intervals_ms.mean(),
        })
    return pd.DataFrame(rows)


def main():
    summary_rows = []

    for condition, modes in CONDITION_MODES.items():
        for mode in modes:
            run_dir = RESULTS_DIR / condition / mode
            if not run_dir.exists():
                continue

            run_frames = []
            for csv_path in sorted(run_dir.glob("run*.csv")):
                run_df = process_run(csv_path)
                if run_df.empty:
                    continue
                run_df["run"] = int(csv_path.stem.replace("run", ""))
                run_frames.append(run_df)

            if not run_frames:
                print(f"[WARN] No data for {condition}/{mode}")
                continue

            all_runs = pd.concat(run_frames, ignore_index=True)

            # Per-run aggregate (mean across all robots in that run)
            per_run = (
                all_runs.groupby("run")[["mean_interval_ms", "std_interval_ms", "mean_hz"]]
                        .mean()
            )

            summary_rows.append({
                "condition":         condition,
                "mode":              mode,
                "n_runs":            len(per_run),
                "interval_mean_ms":  per_run["mean_interval_ms"].mean(),
                "interval_std_ms":   per_run["mean_interval_ms"].std(),
                "jitter_mean_ms":    per_run["std_interval_ms"].mean(),
                "jitter_std_ms":     per_run["std_interval_ms"].std(),   # run-to-run std of jitter
                "hz_mean":           per_run["mean_hz"].mean(),
                "hz_std":            per_run["mean_hz"].std(),
            })

    if not summary_rows:
        print("[WARN] No data found. Run the benchmark first.")
        return

    df = pd.DataFrame(summary_rows)

    print("\n=== /scan inter-arrival statistics (mean ± std across runs) ===\n")
    print(f"{'condition':<15} {'mode':<12} {'runs':>5}  "
          f"{'interval_mean_ms':>18}  {'interval_std_ms':>17}  "
          f"{'jitter_mean_ms':>15}  {'hz_mean':>8}  {'hz_std':>7}")
    print("-" * 100)
    for _, r in df.iterrows():
        print(f"{r['condition']:<15} {r['mode']:<12} {int(r['n_runs']):>5}  "
              f"{r['interval_mean_ms']:>18.3f}  {r['interval_std_ms']:>17.3f}  "
              f"{r['jitter_mean_ms']:>15.3f}  {r['hz_mean']:>8.3f}  {r['hz_std']:>7.4f}")

    out = RESULTS_DIR.parent / "summary_topic_stats.csv"
    df.to_csv(out, index=False)
    print(f"\n[OK] Summary saved to {out}")


if __name__ == "__main__":
    main()
