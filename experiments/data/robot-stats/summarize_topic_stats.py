#!/usr/bin/env python3
"""
Summarize /tf publish-rate and jitter across benchmark conditions.

Only the attested topic (/tf, stored under results/tf/) is analysed; any
other topic directory (e.g. scan) is silently ignored.

Each result file captures one robot's /tf stream over a single long run (~300 s).
Inter-message intervals are computed per robot (wall clock, within each robot's own
message sequence) and the following per-run stats are computed:
  - median_hz       : 1000 / median(interval_ms)
  - interval_std_ms : std of all inter-message intervals  ← jitter signal
  - interval_p95_ms : 95th-percentile interval            ← tail latency

If multiple runs exist for the same parameter combination they are averaged
(mean ± std across runs).

Expected layout:
  results/tf/no_sidecar/baseline/run{N}.csv
  results/tf/with_sidecar/continuous/SSP{ssp}ms-ITERQ{iterq}-cpu{cpu}-run{N}.csv
  columns: robot, topic, ros_stamp_ns, wall_stamp_ns
"""

import re
from pathlib import Path

import numpy as np
import pandas as pd

RESULTS_DIR = Path(__file__).parent / "results"

_TAGGED_RE   = re.compile(
    r"SSP(?P<ssp>\d+)ms-ITERQ(?P<iterq>\d+)-cpu(?P<cpu>[\dp]+|NC)-run(?P<run>\d+)\.csv$",
    re.IGNORECASE,
)
_BASELINE_RE = re.compile(r"^run(?P<run>\d+)\.csv$", re.IGNORECASE)


def _parse_filename(name: str):
    """Return (ssp_ms, iterq, cpu_limit, run_id) or None if unrecognised."""
    m = _TAGGED_RE.search(name)
    if m:
        cpu_str = m.group("cpu").upper()
        cpu_val = None if cpu_str == "NC" else float(cpu_str.replace("P", "."))
        return (
            int(m.group("ssp")),
            int(m.group("iterq")),
            cpu_val,
            int(m.group("run")),
        )
    m = _BASELINE_RE.match(name)
    if m:
        return None, None, None, int(m.group("run"))
    return None


def run_stats(csv_path: Path) -> dict:
    """Per-run stats pooled across all robots (wall-clock intervals, per robot)."""
    df = pd.read_csv(csv_path)
    intervals_ms = []
    for _, grp in df.groupby("robot"):
        grp = grp.sort_values("wall_stamp_ns")
        iv = grp["wall_stamp_ns"].diff().dropna().values / 1e6
        intervals_ms.append(iv[iv > 0])
    if not intervals_ms:
        nan = float("nan")
        return {"median_hz": nan, "interval_std_ms": nan, "interval_p95_ms": nan}
    all_iv = np.concatenate(intervals_ms)
    p25, p50, p75, p95 = np.percentile(all_iv, [25, 50, 75, 95])
    return {
        "median_ms":       float(p50),
        "p25_ms":          float(p25),
        "p75_ms":          float(p75),
        "interval_std_ms": float(np.std(all_iv)),
        "interval_p95_ms": float(p95),
    }


def main():
    if not RESULTS_DIR.exists():
        print("[WARN] results/ not found. Run the benchmark first.")
        return

    records = []
    for csv_path in sorted(RESULTS_DIR.rglob("*.csv")):
        parsed = _parse_filename(csv_path.name)
        if parsed is None:
            print(f"[SKIP] Unrecognised filename: {csv_path.name}")
            continue

        ssp_ms, iterq, cpu_limit, run_id = parsed

        parts = csv_path.parts
        try:
            res_idx = next(i for i, p in enumerate(parts) if p == "results")
        except StopIteration:
            continue
        rel = parts[res_idx + 1:]
        if len(rel) < 4:   # topic / condition / mode / file
            continue
        topic, condition, mode = rel[0], rel[1], rel[2]

        stats = run_stats(csv_path)
        records.append({
            "topic":            topic,
            "condition":        condition,
            "mode":             mode,
            "ssp_ms":           ssp_ms,
            "iterq":            iterq,
            "cpu_limit":        cpu_limit,
            "run":              run_id,
            **stats,
        })

    if not records:
        print("[WARN] No data found.")
        return

    raw = pd.DataFrame(records)
    raw = raw[raw["topic"] != "scan"]
    if raw.empty:
        print("[WARN] No data after filtering out non-attested topics.")
        return

    GROUP = ["topic", "condition", "mode", "ssp_ms", "iterq", "cpu_limit"]

    def agg_col(col):
        return raw.groupby(GROUP, dropna=False)[col].agg(
            **{f"{col}_mean": "mean", f"{col}_std": "std"}
        )

    n_runs = raw.groupby(GROUP, dropna=False)["run"].count().rename("n_runs")
    summary = pd.concat(
        [n_runs, agg_col("median_ms"), agg_col("p25_ms"), agg_col("p75_ms"),
         agg_col("interval_std_ms"), agg_col("interval_p95_ms")],
        axis=1,
    ).reset_index().sort_values(GROUP)

    def _fmt_ssp(r):
        return f"{int(r['ssp_ms'])}" if pd.notna(r['ssp_ms']) else "-"
    def _fmt_iterq(r):
        return f"{int(r['iterq'])}"  if pd.notna(r['iterq'])  else "-"
    def _fmt_cpu(r):
        if pd.isna(r['cpu_limit']) and pd.notna(r['ssp_ms']):
            return "NC"
        return f"{r['cpu_limit']:.2f}" if pd.notna(r['cpu_limit']) else "-"

    print("\n=== Inter-message interval (ms) per condition ===\n")
    hdr = (f"{'topic':<8} {'condition':<14} {'mode':<12} "
           f"{'ssp_ms':>7} {'iterq':>6} {'cpu':>5} "
           f"{'runs':>5}  {'p25_ms':>8}  {'median_ms':>10}  {'p75_ms':>8}  {'std_ms':>8}  {'p95_ms':>8}")
    print(hdr)
    print("-" * len(hdr))
    for _, r in summary.iterrows():
        p25 = f"{r['p25_ms_mean']:>8.3f}"          if pd.notna(r['p25_ms_mean'])          else "     n/a"
        med = f"{r['median_ms_mean']:>10.3f}"       if pd.notna(r['median_ms_mean'])       else "       n/a"
        p75 = f"{r['p75_ms_mean']:>8.3f}"          if pd.notna(r['p75_ms_mean'])          else "     n/a"
        std = f"{r['interval_std_ms_mean']:>8.3f}"  if pd.notna(r['interval_std_ms_mean']) else "     n/a"
        p95 = f"{r['interval_p95_ms_mean']:>8.3f}"  if pd.notna(r['interval_p95_ms_mean']) else "     n/a"
        print(f"{r['topic']:<8} {r['condition']:<14} {r['mode']:<12} "
              f"{_fmt_ssp(r):>7} {_fmt_iterq(r):>6} {_fmt_cpu(r):>5} "
              f"{int(r['n_runs']):>5}  {p25}  {med}  {p75}  {std}  {p95}")

    out = RESULTS_DIR.parent / "summary_topic_stats.csv"
    summary.to_csv(out, index=False)
    print(f"\n[OK] Summary saved to {out}")


if __name__ == "__main__":
    main()
