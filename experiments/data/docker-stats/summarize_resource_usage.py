#!/usr/bin/env python3
"""
Summarize Docker stats across runs (per container), equal-weight only.

- Overall CSV: mean/std across runs (each run contributes equally).
- Timeline CSV: per-second relative-time means across runs (equal weight).
- Run X CSV: per-second relative-time for a specific single run (no averaging).

Input files must be named: <container>-run<idx>.csv
"""

import argparse
from pathlib import Path
import re
import pandas as pd

# Metrics to summarize
METRICS = [
    "cpu_percent",
    "mem_mb",
    "blk_read_mb",
    "blk_write_mb",
    "net_rx_mb",
    "net_tx_mb",
]

FILE_PATTERN = re.compile(r"^(?P<container>.+)-run(?P<run>\d+)\.csv$", re.IGNORECASE)

OUT_OVERALL_FILE = "overall_resource_usage_per_container.csv"
OUT_TIMELINE_FILE = "timeline_resource_usage_per_container.csv"


def parse_args():
    p = argparse.ArgumentParser(description="Summarize docker stats per container across runs (equal weight only).")
    p.add_argument("--base-dir", default=".",
                   help="Base directory (default: current). Usually run from experiments/data/docker-stats/")
    p.add_argument("--input-dir", default="test",
                   help="Subdirectory with per-run CSVs (default: test).")
    p.add_argument("--out-dir", default="_summary",
                   help="Output directory for the summary CSVs (default: _summary).")
    p.add_argument("--export-run", type=int, default=1,
                   help="Specific Run ID to export as a standalone timeline CSV (default: 1).")
    return p.parse_args()


def coerce_numeric(df, cols):
    for c in cols:
        if c in df.columns:
            df[c] = pd.to_numeric(df[c], errors="coerce")
    return df


def main():
    args = parse_args()
    base = Path(args.base_dir).resolve()
    in_dir = (base / args.input_dir).resolve()
    out_dir = (base / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not in_dir.exists():
        raise SystemExit(f"Input directory not found: {in_dir}")

    frames = []
    for csv_path in sorted(in_dir.glob("*.csv")):
        m = FILE_PATTERN.match(csv_path.name)
        if not m:
            continue
        container = m.group("container")
        run_id = int(m.group("run"))

        try:
            df = pd.read_csv(csv_path)
        except Exception as e:
            print(f"[WARN] Skipping {csv_path.name}: {e}")
            continue

        keep = ["timestamp"] + METRICS
        present = [c for c in keep if c in df.columns]
        if not present:
            print(f"[WARN] {csv_path.name} has none of the required columns, skipping.")
            continue

        df = df[present].copy()
        coerce_numeric(df, present)
        df["container"] = container
        df["run"] = run_id
        frames.append(df)

    if not frames:
        raise SystemExit(f"No valid CSVs found in {in_dir} matching '*-run*.csv'.")

    all_df = pd.concat(frames, ignore_index=True)

    # Ensure all metrics exist (create NaNs if missing)
    for m in METRICS:
        if m not in all_df.columns:
            all_df[m] = pd.NA

    # ---------------------------
    # OVERALL SUMMARY (equal-run)
    # ---------------------------
    per_run = (
        all_df.groupby(["container", "run"], dropna=False)[METRICS]
             .mean(numeric_only=True)
             .reset_index()
    )
    overall = per_run.groupby("container", dropna=False).agg({m: ["mean", "std"] for m in METRICS})
    overall.columns = [f"{m}_{stat}" for m, stat in overall.columns]
    overall = overall.reset_index().sort_values("container").reset_index(drop=True)

    out_csv_overall = out_dir / OUT_OVERALL_FILE
    overall.to_csv(out_csv_overall, index=False)
    print(f"[OK] Wrote overall summary: {out_csv_overall}")
    # print(overall.to_string(index=False))

    # ----------------------------------------
    # TIMELINE PREP (relative t)
    # ----------------------------------------
    if "timestamp" not in all_df.columns:
        print("[WARN] No 'timestamp' column found; skipping timeline export.")
        return

    tl = all_df.dropna(subset=["timestamp"]).copy()
    tl["timestamp"] = pd.to_numeric(tl["timestamp"], errors="coerce")
    tl = tl.dropna(subset=["timestamp"])

    # Relative time per run (seconds, rounded)
    t0 = tl.groupby(["container", "run"])["timestamp"].transform("min")
    tl["t_rel_s"] = ((tl["timestamp"] - t0) / 1000.0).round().astype(int)

    # 1) per (container, run, t_rel_s) averages
    # This aligns everyone to seconds (e.g. 1.1s and 1.9s both become second 1)
    per_run_sec = (
        tl.groupby(["container", "run", "t_rel_s"], dropna=False)[METRICS]
          .mean(numeric_only=True)
          .reset_index()
    )

    # ----------------------------------------
    # A) AVERAGED TIMELINE (All Runs)
    # ----------------------------------------
    timeline = (
        per_run_sec.groupby(["container", "t_rel_s"], dropna=False)[METRICS]
                   .mean(numeric_only=True)
                   .reset_index()
                   .sort_values(["container", "t_rel_s"])
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
        # Sort and clean
        run_df = run_df.sort_values(["container", "t_rel_s"]).reset_index(drop=True)
        # Drop the 'run' column since it's just a constant
        run_df = run_df.drop(columns=["run"], errors="ignore")

        out_csv_run = out_dir / f"timeline_resource_usage_per_container_run{target_run}.csv"
        run_df.to_csv(out_csv_run, index=False)
        print(f"[OK] Wrote timeline for Run {target_run}: {out_csv_run}")
    else:
        print(f"[WARN] No data found for Run {target_run}. Available runs: {sorted(per_run_sec['run'].unique())}")


if __name__ == "__main__":
    main()
