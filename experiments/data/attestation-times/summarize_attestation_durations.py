#!/usr/bin/env python3
"""
summarize_attestation_durations.py
Adapted to the high-precision "Sync Pair" architecture (_ns and _ms floats).
Results are expected under:
    results/<N{n_robots}>/<mode>/<prefix>-<participant>-run<N>.json
e.g.  results/N4/startup/startup-robot1-run3.json
      results/N4/continuous/continuous-robot1-run2.json
      results/N4/startup/secaas-run1.json

Output CSVs all carry n_robots and mode columns so downstream plots can
filter/facet without manual path parsing.
"""

import json
from pathlib import Path
import re
import pandas as pd

# --- Configuration ---
BASE_DIR  = Path(".")
INPUT_DIR = BASE_DIR / "results"
OUT_DIR   = BASE_DIR / "_summary"

SUMMARY_FILE       = "durations_summary.csv"
RAW_FILE           = "durations_raw.csv"
PER_RUN_FILE       = "durations_per_run.csv"
EXPORT_EXTRA_FILES = True

# Metric definitions — (metric_name, dur_prefix, start_key, end_key)
# dur_prefix is used to look up {dur_prefix}_ns  or {dur_prefix}_ms first,
# then falls back to wall-clock subtraction (end_key - start_key).
METRICS_CONFIG = {
    "prover": [
        ("e2e_blockchain",      "dur_prover_e2e",             "t_evidence_sent",                          "t_result_received"),
        ("evidence_call",       "dur_send_evidence_call",      "t_prover_start",                           "t_evidence_sent"),
        ("evidence_tx_confirm", "dur_send_evidence_tx_confirm","t_evidence_sent",                          "t_evidence_sent_tx_confirmed"),
        ("total_lifecycle",     "dur_prover_total",            "t_prover_start",                           "t_prover_finished"),
    ],
    "verifier": [
        ("reaction_time",       "dur_verifier_reaction",       "t_ready_for_evaluation_received",          "t_verifier_start"),
        ("signatures_fetch",    "dur_signatures_fetch",        "t_get_signatures_start",                   "t_get_signatures_finished"),
        ("verify_compute",      "dur_verify_compute",          "t_verify_compute_start",                   "t_verify_compute_finished"),
        ("result_call",         "dur_send_result_call",        "t_verify_compute_finished",                "t_result_sent"),
        ("result_tx_confirm",   "dur_send_result_tx_confirm",  "t_result_sent",                            "t_result_sent_tx_confirmed"),
        ("total_lifecycle",     "dur_verifier_total",          "t_verifier_start",                         "t_verifier_finished"),
    ],
    "oracle": [
        ("reaction_time",           "dur_oracle_reaction",                    "t_attestation_started_received",             "t_oracle_start"),
        ("prover_addr_fetch",       "dur_prover_addr_fetch",                  "t_get_prover_addr_start",                    "t_get_prover_addr_finished"),
        ("db_lookup",               "dur_db_fetch",                           "t_get_prover_ref_signatures_db_start",       "t_get_prover_ref_signatures_db_finished"),
        ("ref_signature_call",      "dur_send_prover_ref_signature_call",     "t_get_prover_ref_signatures_db_finished",    "t_prover_ref_signature_sent"),
        ("ref_signature_tx_confirm","dur_send_prover_ref_signature_tx_confirm","t_prover_ref_signature_sent",              "t_prover_ref_signature_sent_tx_confirmed"),
        ("total_lifecycle",         "dur_oracle_total",                       "t_oracle_start",                             "t_oracle_finished"),
    ],
}

# Expected directory layout:  results / N{n} / {mode} / {files}
# Files named:  {mode}-{participant}-run{N}.json   (robots)
#               secaas-run{N}.json                 (oracle)
# The mode prefix in the filename is stripped before participant extraction.
_FILE_PATTERN = re.compile(r"^(?P<participant>.+)-run(?P<run>\d+)\.json$", re.IGNORECASE)
_KNOWN_MODES  = {"startup", "continuous"}


def _parse_path(json_path: Path):
    """
    Returns (n_robots, mode, participant, run_id) from a path like
        results/N4/continuous/continuous-robot1-run2.json
    or  results/N4/startup/secaas-run5.json
    Returns None if the path doesn't match the expected layout.
    """
    parts = json_path.parts  # (..., "results", "N4", "continuous", "filename.json")
    try:
        results_idx = next(i for i, p in enumerate(parts) if p == "results")
    except StopIteration:
        return None

    rel = parts[results_idx + 1:]  # ("N4", "continuous", "filename.json")
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

    m = _FILE_PATTERN.match(filename)
    if not m:
        return None

    raw_participant = m.group("participant")
    run_id          = int(m.group("run"))

    # Strip the mode prefix from the participant name when present
    # e.g. "startup-robot1" → "robot1",  "continuous-robot2" → "robot2"
    # "secaas" stays as-is
    for prefix in _KNOWN_MODES:
        if raw_participant.lower().startswith(prefix + "-"):
            raw_participant = raw_participant[len(prefix) + 1:]
            break

    participant = _clean_label(raw_participant)
    return n_robots, mode, participant, run_id


def _clean_label(raw: str) -> str:
    raw = raw.strip()
    if raw.lower() == "secaas":
        return "SECaaS"
    if raw.lower().startswith("robot"):
        return "Robot" + raw.lower().replace("robot", "")
    return raw.title()


def _participant_group(participant: str) -> str:
    p = participant.strip()
    if p.lower().startswith("robot"):
        return "Robot"
    if p.lower() == "secaas":
        return "SECaaS"
    return p


def _safe_get(d, key):
    try:
        val = d.get(key)
        return float(val) if val is not None else None
    except Exception:
        return None


def main():
    in_dir  = INPUT_DIR.resolve()
    out_dir = OUT_DIR.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    data_points = []
    print(f"Reading JSONs from: {in_dir}")

    for json_path in sorted(in_dir.rglob("*.json")):
        parsed = _parse_path(json_path)
        if parsed is None:
            continue

        n_robots, mode, participant, run_id = parsed

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except Exception:
            continue

        for role, metrics in METRICS_CONFIG.items():
            entries = doc.get(role, [])
            if not isinstance(entries, list):
                continue

            for (metric_name, dur_prefix, start_key, end_key) in metrics:
                for entry in entries:
                    val_ms = None

                    # 1. Native nanoseconds (highest precision)
                    val_ns = _safe_get(entry, f"{dur_prefix}_ns")
                    if val_ns is not None:
                        val_ms = val_ns / 1_000_000.0
                    else:
                        # 2. Pre-calculated high-precision ms
                        val_ms = _safe_get(entry, f"{dur_prefix}_ms")

                        # 3. Wall-clock timestamp subtraction
                        if val_ms is None:
                            t_s = _safe_get(entry, start_key)
                            t_e = _safe_get(entry, end_key)
                            if t_s is not None and t_e is not None:
                                val_ms = t_e - t_s

                    if val_ms is not None and val_ms >= 0:
                        data_points.append({
                            "n_robots":    n_robots,
                            "mode":        mode,
                            "participant": participant,
                            "run":         run_id,
                            "role":        role,
                            "metric":      metric_name,
                            "duration_s":  val_ms / 1000.0,
                        })

    if not data_points:
        print("[WARN] No data found.")
        return

    df = pd.DataFrame(data_points)

    GROUP_KEYS   = ["n_robots", "mode", "participant", "run", "role", "metric"]
    SUMMARY_KEYS = ["n_robots", "mode", "participant", "participant_group", "role", "metric"]

    AGG = dict(
        run_count="count",
        mean_s="mean",
        std_s="std",
        min_s="min",
        max_s="max",
        median_s="median",
        p25_s=lambda x: x.quantile(0.25),
        p75_s=lambda x: x.quantile(0.75),
    )

    if EXPORT_EXTRA_FILES:
        raw_path = out_dir / RAW_FILE
        df.to_csv(raw_path, index=False)
        print(f"[OK] Raw data saved to:     {raw_path}")

    # Per-run mean (equal weight per run in summary)
    per_run = (
        df.groupby(GROUP_KEYS, as_index=False)["duration_s"]
        .mean()
        .rename(columns={"duration_s": "run_mean_s"})
    )
    per_run["participant_group"] = per_run["participant"].apply(_participant_group)

    if EXPORT_EXTRA_FILES:
        per_run_path = out_dir / PER_RUN_FILE
        per_run.to_csv(per_run_path, index=False)
        print(f"[OK] Per-run means saved to: {per_run_path}")

    # Single summary: keeps individual participant AND group column so plots
    # can use either granularity (Robot1/Robot2/... or Robot/SECaaS) from
    # the same file without reloading.
    summary = (
        per_run.groupby(SUMMARY_KEYS)["run_mean_s"]
        .agg(**AGG)
        .reset_index()
    )

    sum_path = out_dir / SUMMARY_FILE
    summary.to_csv(sum_path, index=False)
    print(f"[OK] Summary saved to:       {sum_path}")


if __name__ == "__main__":
    main()
