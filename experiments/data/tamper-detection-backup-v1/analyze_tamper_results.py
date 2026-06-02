#!/usr/bin/env python3
"""
Tamper Detection Latency — Statistical Analysis

Reads all SSPNms-batch*.csv files from:
  experiments/data/tamper-detection/results/<target>/

Produces:
  _summary/<target>-summary.csv   — per-SSP statistics (one file per target)

For paper-quality plots use:
  python3 experiments/plot/boxplot_tamper_detection.py --target both

Usage:
  python3 analyze_tamper_results.py                  # both targets (default)
  python3 analyze_tamper_results.py --target state_publisher
  python3 analyze_tamper_results.py --target sidecar
"""

import argparse
import csv
import statistics
from pathlib import Path

import numpy as np

# ── Paths ─────────────────────────────────────────────────────────────────────
HERE        = Path(__file__).parent.resolve()
RESULTS_DIR = HERE / "results"
SUMMARY_DIR = HERE / "_summary"

# ── Data loading ──────────────────────────────────────────────────────────────

def load_target(target: str) -> tuple[dict, dict]:
    """
    Returns:
      latencies  — {ssp_ms: [latency_s, ...]}  (detected trials only)
      totals     — {ssp_ms: int}                (all trials including not-detected)
    """
    target_dir = RESULTS_DIR / target
    if not target_dir.exists():
        return {}, {}

    latencies: dict[int, list[float]] = {}
    totals:    dict[int, int]         = {}

    for f in sorted(target_dir.glob("SSP*ms-batch*.csv")):
        with f.open(encoding="utf-8") as fh:
            for row in csv.DictReader(fh):
                ssp = int(row["ssp_ms"])
                totals[ssp] = totals.get(ssp, 0) + 1
                if row["detected"] == "1" and row["latency_s"]:
                    latencies.setdefault(ssp, []).append(float(row["latency_s"]))

    return latencies, totals


# ── Statistics ────────────────────────────────────────────────────────────────

def summarize(latencies: dict, totals: dict) -> list[dict]:
    rows = []
    for ssp in sorted(latencies.keys()):
        lats    = latencies[ssp]
        n_det   = len(lats)
        n_total = totals.get(ssp, n_det)
        rows.append({
            "ssp_ms":         ssp,
            "n_total":        n_total,
            "n_detected":     n_det,
            "detection_rate": round(n_det / n_total, 4) if n_total else 0.0,
            "mean_s":         round(statistics.mean(lats),   3),
            "median_s":       round(statistics.median(lats), 3),
            "std_s":          round(statistics.stdev(lats) if n_det > 1 else 0.0, 3),
            "min_s":          round(min(lats), 3),
            "max_s":          round(max(lats), 3),
            "p25_s":          round(float(np.percentile(lats, 25)), 3),
            "p75_s":          round(float(np.percentile(lats, 75)), 3),
        })
    return rows


def print_table(rows: list[dict], target: str) -> None:
    print(f"\n{'═'*78}")
    print(f"  Tamper Detection Results — target: {target}")
    print(f"{'═'*78}")
    print(f"  {'SSP':>8}  {'N':>4}  {'DetRate':>8}  {'Mean':>7}  "
          f"{'Median':>7}  {'Std':>6}  {'Min':>6}  {'Max':>6}  {'IQR':>14}")
    print(f"  {'-'*76}")
    for r in rows:
        iqr = round(r["p75_s"] - r["p25_s"], 3)
        dr  = f"{r['detection_rate']*100:.0f}%"
        print(f"  {r['ssp_ms']:>6}ms  {r['n_detected']:>4}  {dr:>8}  "
              f"{r['mean_s']:>6.3f}s  {r['median_s']:>6.3f}s  "
              f"{r['std_s']:>5.3f}s  {r['min_s']:>5.3f}s  {r['max_s']:>5.3f}s  "
              f"[{r['p25_s']:.3f}–{r['p75_s']:.3f}]s")

    # Dynamic observations
    all_det = all(r["detection_rate"] == 1.0 for r in rows)
    floor_row = min(rows, key=lambda r: r["min_s"])
    floor = floor_row["min_s"]
    floor_ssp = floor_row["ssp_ms"]

    print(f"\n  Observations:")
    print(f"  • Detection rate: {'100%' if all_det else 'partial'} across all SSP values")
    print(f"  • Blockchain confirmation floor: ≈{floor:.2f} s "
          f"(min latency observed at SSP={floor_ssp} ms)")
    for r in rows:
        ratio = r["mean_s"] / (r["ssp_ms"] / 1000)
        print(f"  • SSP={r['ssp_ms']:>6} ms → mean {r['mean_s']:.3f} s  "
              f"({ratio:.2f}× SSP,  floor fraction = {floor/r['mean_s']*100:.0f}%)")


def print_comparison(results: dict[str, list[dict]]) -> None:
    """Side-by-side mean comparison when both targets are available."""
    targets = list(results.keys())
    if len(targets) < 2:
        return

    ssps = sorted({r["ssp_ms"] for rows in results.values() for r in rows})
    t0, t1 = targets[0], targets[1]

    print(f"\n{'═'*78}")
    print(f"  Comparison — {t0}  vs  {t1}")
    print(f"{'═'*78}")
    print(f"  {'SSP':>8}   {'mean '+t0:>22}   {'mean '+t1:>22}   {'Δ mean':>8}")
    print(f"  {'-'*76}")

    def _mean(rows, ssp):
        for r in rows:
            if r["ssp_ms"] == ssp:
                return r["mean_s"]
        return None

    for ssp in ssps:
        m0 = _mean(results[t0], ssp)
        m1 = _mean(results[t1], ssp)
        if m0 is None or m1 is None:
            continue
        delta = m1 - m0
        print(f"  {ssp:>6} ms   {m0:>20.3f} s   {m1:>20.3f} s   "
              f"{delta:>+7.3f} s")
    print()


# ── CSV export ────────────────────────────────────────────────────────────────

def save_summary_csv(rows: list[dict], target: str) -> None:
    SUMMARY_DIR.mkdir(parents=True, exist_ok=True)
    out = SUMMARY_DIR / f"{target}-summary.csv"
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)
    print(f"\n  → {out.relative_to(HERE.parent.parent.parent)}")


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    p = argparse.ArgumentParser(
        description="Analyse tamper-detection latency CSVs.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument(
        "--target",
        choices=["state_publisher", "sidecar", "both"],
        default="both",
        help="Which target's results to analyse. Default: both",
    )
    args = p.parse_args()

    targets = (
        ["state_publisher", "sidecar"] if args.target == "both" else [args.target]
    )

    all_results: dict[str, list[dict]] = {}

    for target in targets:
        latencies, totals = load_target(target)
        if not latencies:
            print(f"\n[!] No data found for target={target} in {RESULTS_DIR / target}")
            continue
        rows = summarize(latencies, totals)
        print_table(rows, target)
        save_summary_csv(rows, target)
        all_results[target] = rows

    if len(all_results) == 2:
        print_comparison(all_results)

    print(f"\n  Paper-quality plot:")
    print(f"    python3 experiments/plot/boxplot_tamper_detection.py --target both")


if __name__ == "__main__":
    main()
