#!/usr/bin/env python3
"""
Per-N summary of blockchain stats for continuous-mode experiments.

Expected file layout:
    results/N{n}/continuous/{variant}/blockchain-SSP{ssp}s-ITERQ{iterq}-cpu{cpu}-run{k}.csv
e.g.
    results/N4/continuous/rr/blockchain-SSP20s-ITERQ1-cpu0p4-run1.csv

Aggregation pipeline:
  blocks (per run CSV) → per-run stats → mean ± std across (N, ssp_s, iterq, cpu_limit) groups
"""
import argparse
import glob
import os
import re

import pandas as pd

RESULTS_ROOT    = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
N_VALUES        = [4, 8, 16, 24, 32, 40, 50, 64, 100]
_KNOWN_VARIANTS = {"rr", "lv"}

SUMMARY_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_summary")

_FILE_RE   = re.compile(r"^(?P<base>.+)-run(?P<run>\d+)\.csv$", re.IGNORECASE)
_PARAMS_RE = re.compile(
    r"-SSP(?P<ssp>\d+)s-ITERQ(?P<iterq>\d+)-cpu(?P<cpu>[\dp]+)$",
    re.IGNORECASE,
)


def _extract_params(base: str):
    """Return (ssp_s, iterq, cpu_limit) parsed from the base stem, or (None, None, None)."""
    m = _PARAMS_RE.search(base)
    if m:
        return int(m.group("ssp")), int(m.group("iterq")), float(m.group("cpu").replace("p", "."))
    return None, None, None


def parse_args():
    p = argparse.ArgumentParser(description="Summarize blockchain stats per N.")
    p.add_argument("--variant", default="rr",
                   choices=sorted(_KNOWN_VARIANTS),
                   help="Contract variant to summarize (default: rr).")
    return p.parse_args()


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


def summarize_variant(variant: str) -> list[dict]:
    """Scan all blockchain CSVs under variant dir; return one row per (N, ssp_s, iterq, cpu_limit)."""
    rows = []
    for n in N_VALUES:
        dir_path = os.path.join(RESULTS_ROOT, f"N{n}", "continuous", variant)
        files    = sorted(glob.glob(os.path.join(dir_path, "blockchain-*.csv")))
        if not files:
            continue

        # Group files by (ssp_s, iterq, cpu_limit)
        groups: dict[tuple, list] = {}
        for f in files:
            m = _FILE_RE.match(os.path.basename(f))
            if not m:
                continue
            ssp_s, iterq, cpu_limit = _extract_params(m.group("base"))
            groups.setdefault((ssp_s, iterq, cpu_limit), []).append(f)

        for (ssp_s, iterq, cpu_limit), group_files in sorted(groups.items()):
            run_stats = []
            for f in group_files:
                try:
                    run_stats.append(per_run_stats(pd.read_csv(f)))
                except Exception as e:
                    print(f"[WARN] Skipping {f}: {e}")
            if not run_stats:
                continue
            runs_df = pd.DataFrame(run_stats)
            row = {"N": n, "ssp_s": ssp_s, "iterq": iterq, "cpu_limit": cpu_limit, "runs": len(run_stats)}
            for col in runs_df.columns:
                row[f"{col}_mean"] = runs_df[col].mean()
                row[f"{col}_std"]  = runs_df[col].std()
            rows.append(row)
    return rows


def main():
    args    = parse_args()
    variant = args.variant

    rows = summarize_variant(variant)
    if not rows:
        print(f"No data found for variant '{variant}'.")
        return

    df = pd.DataFrame(rows)

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
        ("bytes_per_s",    "growth (B/s)"),
    ]

    print(f"\nVariant: {variant}")
    header = (f"{'N':>5}  {'ssp_s':>5}  {'iterq':>6}  {'cpu':>5}  {'runs':>4}"
              + "".join(f"  {lbl:>{COL_W}}" for _, lbl in metrics))
    print(header)
    print("-" * len(header))

    for _, row in df.iterrows():
        line = (f"{int(row['N']):>5}  {str(row['ssp_s']):>5}  "
                f"{str(row['iterq']):>6}  {str(row['cpu_limit']):>5}  {int(row['runs']):>4}")
        for key, _ in metrics:
            mean = row[f"{key}_mean"]
            std  = row[f"{key}_std"]
            val  = f"{mean:.2f} ± {std:.2f}"
            line += f"  {val:>{COL_W}}"
        print(line)

    print()

    # ---- save CSV ----
    os.makedirs(SUMMARY_DIR, exist_ok=True)
    out = os.path.join(SUMMARY_DIR, f"blockchain_stats_summary_{variant}.csv")
    df.to_csv(out, index=False)
    print(f"Summary saved to {out}")


if __name__ == "__main__":
    main()
