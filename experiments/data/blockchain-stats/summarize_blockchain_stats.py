#!/usr/bin/env python3
"""
Per-condition summary of blockchain stats for continuous-mode experiments.

Expected file layout:
    results/N{n}/blockchain-SSP{ssp_ms}ms-ITERQ{iterq}-run{k}.csv
    results/idle/blockchain-idle-*.csv          (baseline, no N/SSP/ITERQ)
e.g.
    results/N64/blockchain-SSP20000ms-ITERQ1-run1.csv
    results/idle/blockchain-idle-300s.csv
"""
import glob
import os
import re

import pandas as pd

RESULTS_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), "results")
N_VALUES     = [4, 8, 16, 24, 32, 40, 50, 64, 100]

SUMMARY_DIR  = os.path.join(os.path.dirname(os.path.abspath(__file__)), "_summary")

_FILE_RE   = re.compile(r"^(?P<base>.+)-run(?P<run>\d+)\.csv$", re.IGNORECASE)
_PARAMS_RE = re.compile(r"-SSP(?P<ssp>\d+)ms-ITERQ(?P<iterq>\d+)$", re.IGNORECASE)


def _extract_params(base: str):
    m = _PARAMS_RE.search(base)
    if m:
        return int(m.group("ssp")), int(m.group("iterq"))
    return None, None


def per_run_stats(df: pd.DataFrame) -> dict:
    duration_s = df["block_timestamp"].max() - df["block_timestamp"].min()
    if duration_s <= 0:
        duration_s = 120

    total_gas = df["gas_used"].sum()
    total_tx  = df["tx_count"].sum()
    bt        = df["block_time_s"].dropna()

    return {
        "tx_per_block":   df["tx_count"].mean(),
        "total_tx":       float(total_tx),
        "gas_used_pct":   df["gas_used_pct"].mean(),
        "avg_gas_per_tx": total_gas / total_tx if total_tx > 0 else 0.0,
        "block_time_s":   bt.mean(),
        "block_time_std": bt.std(),
        "total_bytes":    df["size_bytes"].sum(),
        "bytes_per_s":    df["size_bytes"].sum() / duration_s,
    }


def collect_rows() -> list[dict]:
    # Group files by (N, ssp_ms, iterq); keep only the highest-indexed run.
    best: dict[tuple, tuple[int, str]] = {}  # key → (run_index, path)
    for n in N_VALUES:
        dir_path = os.path.join(RESULTS_ROOT, f"N{n}")
        for f in sorted(glob.glob(os.path.join(dir_path, "blockchain-*.csv"))):
            m = _FILE_RE.match(os.path.basename(f))
            if not m:
                continue
            ssp_ms, iterq = _extract_params(m.group("base"))
            if ssp_ms is None:
                continue
            key = (n, ssp_ms, iterq)
            run = int(m.group("run"))
            if run > best.get(key, (-1, ""))[0]:
                best[key] = (run, f)

    rows = []
    for (n, ssp_ms, iterq), (run, f) in sorted(best.items()):
        try:
            stats = per_run_stats(pd.read_csv(f))
        except Exception as e:
            print(f"[WARN] Skipping {f}: {e}")
            continue
        rows.append({"N": n, "ssp_ms": ssp_ms, "iterq": iterq, "run": run, **stats})
    return rows


def collect_idle() -> dict | None:
    """Return stats for the most recent idle baseline CSV, or None if absent."""
    files = sorted(glob.glob(os.path.join(RESULTS_ROOT, "idle", "blockchain-idle-*.csv")))
    if not files:
        return None
    f = files[-1]   # latest alphabetically (longer duration wins ties)
    try:
        return {"file": os.path.basename(f), **per_run_stats(pd.read_csv(f))}
    except Exception as e:
        print(f"[WARN] Skipping idle file {f}: {e}")
        return None


def _fmt_key(val) -> str:
    try:
        return str(int(val))
    except (ValueError, TypeError):
        return str(val)


def print_table(df: pd.DataFrame, metrics: list[tuple], key_cols: list[str], col_w: int = 18):
    header = "  ".join(f"{c:>{col_w}}" for c in key_cols)
    header += "".join(f"  {lbl:>{col_w}}" for _, lbl in metrics)
    print(header)
    print("-" * len(header))
    for _, row in df.iterrows():
        line = "  ".join(f"{_fmt_key(row[c]):>{col_w}}" for c in key_cols)
        for key, _ in metrics:
            line += f"  {row[key]:>{col_w}.2f}"
        print(line)


def main():
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

    os.makedirs(SUMMARY_DIR, exist_ok=True)

    # ── Idle baseline ──────────────────────────────────────────────────────────
    idle = collect_idle()
    if idle:
        print("\n=== Idle baseline ===")
        print(f"File: {idle['file']}")
        idle_df = pd.DataFrame([idle])
        for key, lbl in metrics:
            print(f"  {lbl:<20} {idle_df[key].iloc[0]:.2f}")
        out_idle = os.path.join(SUMMARY_DIR, "blockchain_stats_idle.csv")
        idle_df.to_csv(out_idle, index=False)
        print(f"Saved to {out_idle}")
    else:
        print("\n[INFO] No idle baseline found in results/idle/")

    # ── Continuous runs ────────────────────────────────────────────────────────
    rows = collect_rows()
    if not rows:
        print("\nNo continuous-run data found.")
        return

    df = pd.DataFrame(rows)

    print("\n=== Continuous runs ===")
    print_table(df, metrics, key_cols=["N", "ssp_ms", "iterq", "run"])
    print()

    out = os.path.join(SUMMARY_DIR, "blockchain_stats_summary.csv")
    df.to_csv(out, index=False)
    print(f"Summary saved to {out}")


if __name__ == "__main__":
    main()
