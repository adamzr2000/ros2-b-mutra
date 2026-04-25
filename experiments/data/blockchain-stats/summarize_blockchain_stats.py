#!/usr/bin/env python3
"""
Per-N summary of blockchain stats for continuous-mode experiments.

Aggregation pipeline:
  blocks (per run CSV) → per-run stats → mean ± std across runs → one row per N
"""
import glob
import os

import numpy as np
import pandas as pd

RESULTS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
N_VALUES     = [4, 8, 16, 24, 32, 40, 50, 64, 100]


def per_run_stats(df: pd.DataFrame) -> dict:
    duration_s = df["block_timestamp"].max() - df["block_timestamp"].min()
    if duration_s <= 0:
        duration_s = 120

    total_gas = df["gas_used"].sum()
    total_tx  = df["tx_count"].sum()
    bt        = df["block_time_s"].dropna()

    return {
        "tx_per_block":    df["tx_count"].mean(),
        "total_tx":        float(total_tx),
        "gas_used_pct":    df["gas_used_pct"].mean(),
        "avg_gas_per_tx":  total_gas / total_tx if total_tx > 0 else 0.0,
        "block_time_s":    bt.mean(),
        "block_time_std":  bt.std(),
        "total_bytes":     df["size_bytes"].sum(),
        "bytes_per_s":     df["size_bytes"].sum() / duration_s,
    }


def summarize_n(n: int) -> dict | None:
    pattern = os.path.join(RESULTS_ROOT, f"N{n}", "continuous", "blockchain-run*.csv")
    files   = sorted(glob.glob(pattern))
    if not files:
        return None

    runs = pd.DataFrame([per_run_stats(pd.read_csv(f)) for f in files])
    row  = {"N": n, "runs": len(files)}
    for col in runs.columns:
        row[f"{col}_mean"] = runs[col].mean()
        row[f"{col}_std"]  = runs[col].std()
    return row


SUMMARY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_summary")


def main():
    rows = [r for n in N_VALUES if (r := summarize_n(n)) is not None]
    if not rows:
        print("No data found.")
        return

    df = pd.DataFrame(rows).set_index("N")

    # ---- console table ----
    COL_W = 22
    metrics = [
        ("tx_per_block",   "tx/block"),
        ("total_tx",       "total_tx"),
        ("gas_used_pct",   "gas_used %"),
        ("avg_gas_per_tx", "gas/tx"),
        ("block_time_s",   "block_time (s)"),
        ("block_time_std", "blk_time_std (s)"),
        ("total_bytes",    "total_bytes"),
        ("bytes_per_s",    "bytes/s"),
    ]

    header = f"{'N':>5}  {'runs':>4}" + "".join(f"  {lbl:>{COL_W}}" for _, lbl in metrics)
    print("\n" + header)
    print("-" * len(header))

    for n, row in df.iterrows():
        line = f"{n:>5}  {int(row['runs']):>4}"
        for key, _ in metrics:
            mean = row[f"{key}_mean"]
            std  = row[f"{key}_std"]
            val  = f"{mean:.2f} ± {std:.2f}"
            line += f"  {val:>{COL_W}}"
        print(line)

    print()

    # ---- save CSV ----
    os.makedirs(SUMMARY_DIR, exist_ok=True)
    out = os.path.join(SUMMARY_DIR, "blockchain_stats_summary.csv")
    df.to_csv(out)
    print(f"Summary saved to {out}")


if __name__ == "__main__":
    main()
