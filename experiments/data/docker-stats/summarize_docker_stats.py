#!/usr/bin/env python3
"""
Summarize Docker stats across runs (per container), equal-weight only.

Expected file layout:
    results/N{n_robots}/{mode}/{container}-run{N}.csv
e.g.
    results/N4/continuous/robot1-sidecar-run1.csv
    results/N4/startup/secaas-run3.csv

Outputs (all include n_robots and mode columns):
  - overall_resource_usage_per_container.csv   : mean/std per container across runs
  - timeline_resource_usage_per_container.csv  : per-second relative-time means across runs
  - timeline_resource_usage_per_container_run{X}.csv : single-run timeline
"""

import argparse
from pathlib import Path
import re
import pandas as pd

METRICS = [
    "cpu_percent",
    "mem_mb",
    "blk_read_mb",
    "blk_write_mb",
    "net_rx_mb",
    "net_tx_mb",
]

FILE_PATTERN   = re.compile(r"^(?P<container>.+)-run(?P<run>\d+)\.csv$", re.IGNORECASE)
_KNOWN_MODES   = {"startup", "continuous"}

OUT_OVERALL_FILE  = "overall_resource_usage_per_container.csv"
OUT_TIMELINE_FILE = "timeline_resource_usage_per_container.csv"


def parse_args():
    p = argparse.ArgumentParser(
        description="Summarize docker stats per container across runs (equal weight only).")
    p.add_argument("--base-dir",    default=".",
                   help="Base directory (default: current).")
    p.add_argument("--input-dir",   default="results",
                   help="Subdirectory with per-run CSVs (default: results).")
    p.add_argument("--out-dir",     default="_summary",
                   help="Output directory for the summary CSVs (default: _summary).")
    p.add_argument("--export-run",  type=int, default=1,
                   help="Specific Run ID to export as a standalone timeline CSV (default: 1).")
    return p.parse_args()


def _parse_path(csv_path: Path):
    """
    Returns (n_robots, mode, container, run_id) from a path like
        results/N4/continuous/robot1-sidecar-run2.csv
    Returns None if the path doesn't match the expected layout.
    """
    parts = csv_path.parts
    try:
        results_idx = next(i for i, p in enumerate(parts) if p == "results")
    except StopIteration:
        return None

    rel = parts[results_idx + 1:]   # ("N4", "continuous", "filename.csv")
    if len(rel) != 3:
        return None

    n_label, mode, filename = rel

    if not n_label.upper().startswith("N"):
        return None
    if mode not in _KNOWN_MODES:
        return None

    try:
        n_robots = int(n_label[1:])
    except ValueError:
        return None

    m = FILE_PATTERN.match(filename)
    if not m:
        return None

    container = m.group("container")
    run_id    = int(m.group("run"))
    return n_robots, mode, container, run_id


def coerce_numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def main():
    args   = parse_args()
    base   = Path(args.base_dir).resolve()
    in_dir = (base / args.input_dir).resolve()
    out_dir = (base / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not in_dir.exists():
        raise SystemExit(f"Input directory not found: {in_dir}")

    frames = []
    for csv_path in sorted(in_dir.rglob("*.csv")):
        parsed = _parse_path(csv_path)
        if parsed is None:
            continue
        n_robots, mode, container, run_id = parsed

        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f"[WARN] Skipping {csv_path.name}: {e}")
            continue

        keep    = ["timestamp"] + METRICS
        present = [c for c in keep if c in df.columns]
        if not present:
            print(f"[WARN] {csv_path.name} has none of the required columns, skipping.")
            continue

        df = df[present].copy()
        coerce_numeric(df, present)
        df["n_robots"]  = n_robots
        df["mode"]      = mode
        df["container"] = container
        df["run"]       = run_id
        frames.append(df)

    if not frames:
        raise SystemExit(f"No valid CSVs found under {in_dir} matching N{{n}}/{{mode}}/*-run*.csv")

    all_df = pd.concat(frames, ignore_index=True)

    # Ensure all metrics exist (fill with NaN if missing)
    for metric in METRICS:
        if metric not in all_df.columns:
            all_df[metric] = pd.NA

    GROUP_KEYS = ["n_robots", "mode", "container", "run"]

    # ---------------------------
    # OVERALL SUMMARY (equal-run)
    # ---------------------------
    per_run = (
        all_df.groupby(GROUP_KEYS, dropna=False)[METRICS]
              .mean(numeric_only=True)
              .reset_index()
    )
    overall = (
        per_run.groupby(["n_robots", "mode", "container"], dropna=False)
               .agg({m: ["mean", "std"] for m in METRICS})
    )
    overall.columns = [f"{m}_{stat}" for m, stat in overall.columns]
    overall = overall.reset_index().sort_values(["n_robots", "mode", "container"]).reset_index(drop=True)

    out_csv_overall = out_dir / OUT_OVERALL_FILE
    overall.to_csv(out_csv_overall, index=False)
    print(f"[OK] Wrote overall summary:          {out_csv_overall}")

    # ----------------------------------------
    # TIMELINE PREP (relative t)
    # ----------------------------------------
    if "timestamp" not in all_df.columns:
        print("[WARN] No 'timestamp' column found; skipping timeline export.")
        return

    tl = all_df.dropna(subset=["timestamp"]).copy()
    tl["timestamp"] = pd.to_numeric(tl["timestamp"], errors="coerce")
    tl = tl.dropna(subset=["timestamp"])

    # Relative time per (n_robots, mode, container, run), rounded to seconds
    t0 = tl.groupby(GROUP_KEYS)["timestamp"].transform("min")
    tl["t_rel_s"] = ((tl["timestamp"] - t0) / 1000.0).round().astype(int)

    per_run_sec = (
        tl.groupby(GROUP_KEYS + ["t_rel_s"], dropna=False)[METRICS]
          .mean(numeric_only=True)
          .reset_index()
    )

    # ----------------------------------------
    # A) AVERAGED TIMELINE (all runs)
    # ----------------------------------------
    timeline = (
        per_run_sec.groupby(["n_robots", "mode", "container", "t_rel_s"], dropna=False)[METRICS]
                   .mean(numeric_only=True)
                   .reset_index()
                   .sort_values(["n_robots", "mode", "container", "t_rel_s"])
                   .reset_index(drop=True)
    )

    out_csv_timeline = out_dir / OUT_TIMELINE_FILE
    timeline.to_csv(out_csv_timeline, index=False)
    print(f"[OK] Wrote averaged timeline summary: {out_csv_timeline}")

    # ----------------------------------------
    # B) SPECIFIC RUN TIMELINE (Run X only)
    # ----------------------------------------
    target_run = args.export_run
    run_df = per_run_sec[per_run_sec["run"] == target_run].copy()

    if not run_df.empty:
        run_df = (
            run_df.drop(columns=["run"], errors="ignore")
                  .sort_values(["n_robots", "mode", "container", "t_rel_s"])
                  .reset_index(drop=True)
        )
        out_csv_run = out_dir / f"timeline_resource_usage_per_container_run{target_run}.csv"
        run_df.to_csv(out_csv_run, index=False)
        print(f"[OK] Wrote timeline for Run {target_run}:        {out_csv_run}")
    else:
        print(f"[WARN] No data for Run {target_run}. Available: {sorted(per_run_sec['run'].unique())}")


if __name__ == "__main__":
    main()
