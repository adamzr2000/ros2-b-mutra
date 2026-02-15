#!/usr/bin/env python3
"""
summarize_attestation_durations.py
Updates: Adapted to the high-precision "Sync Pair" architecture (_ns and _ms floats).
"""

import json
from pathlib import Path
import re
from typing import Optional, List, Dict, Any
import pandas as pd
import numpy as np

# --- Configuration ---
BASE_DIR = Path(".")
INPUT_DIR = BASE_DIR / "results"
OUT_DIR = BASE_DIR / "_summary"

SUMMARY_FILE = "attestation_durations_summary.csv"
RAW_FILE = "attestation_durations_raw.csv"

# Updated to match the new ROLE_PRECISION_MAP prefixes in helpers.py/go
METRICS_CONFIG = {
    "prover": [
        ("e2e_blockchain", "dur_prover_e2e", "t_evidence_sent", "t_result_received"),
        ("evidence_call", "dur_send_evidence_call", "t_prover_start", "t_evidence_sent"),
        ("evidence_tx_confirm", "dur_send_evidence_tx_confirm", "t_evidence_sent", "t_evidence_sent_tx_confirmed"),
        ("total_lifecycle", "dur_prover_total", "t_prover_start", "t_prover_finished"),
    ],
    "verifier": [
        ("reaction_time", "dur_verifier_reaction", "t_ready_for_evaluation_received", "t_verifier_start"),
        ("signatures_fetch", "dur_signatures_fetch", "t_get_signatures_start", "t_get_signatures_finished"),
        ("verify_compute", "dur_verify_compute", "t_verify_compute_start", "t_verify_compute_finished"),
        ("result_call", "dur_send_result_call", "t_verify_compute_finished", "t_result_sent"),
        ("result_tx_confirm", "dur_send_result_tx_confirm", "t_result_sent", "t_result_sent_tx_confirmed"),
        ("total_lifecycle", "dur_verifier_total", "t_verifier_start", "t_verifier_finished"),
    ],
    "oracle": [
        ("reaction_time", "dur_oracle_reaction", "t_attestation_started_received", "t_oracle_start"),
        ("prover_addr_fetch", "dur_prover_addr_fetch", "t_get_prover_addr_start", "t_get_prover_addr_finished"),
        ("db_lookup", "dur_db_fetch", "t_get_prover_ref_signatures_db_start", "t_get_prover_ref_signatures_db_finished"),
        ("ref_signature_call", "dur_send_prover_ref_signature_call", "t_get_prover_ref_signatures_db_finished", "t_prover_ref_signatures_sent"),
        ("ref_signature_tx_confirm", "dur_send_prover_ref_signatures_tx_confirm", "t_prover_ref_signatures_sent", "t_prover_ref_signatures_sent_tx_confirmed"),
        ("total_lifecycle", "dur_oracle_total", "t_oracle_start", "t_oracle_finished"),
    ]
}

FILE_PATTERN = re.compile(r"^(?P<participant>.+)-run(?P<run>\d+)\.json$", re.IGNORECASE)

def _clean_label(raw: str) -> str:
    raw = raw.strip()
    if raw.lower() == "secaas": return "SECaaS"
    if raw.lower().startswith("robot"): return "Robot" + raw.lower().replace("robot", "")
    return raw.title()

def _safe_get(d, key):
    try:
        val = d.get(key)
        return float(val) if val is not None else None
    except:
        return None

def main():
    in_dir = INPUT_DIR.resolve()
    out_dir = OUT_DIR.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    data_points = []
    print(f"Reading JSONs from: {in_dir}")
    # Using rglob to catch json files even if they are in subdirectories
    found_files = sorted(in_dir.rglob("*.json"))

    for json_path in found_files:
        m = FILE_PATTERN.match(json_path.name)
        if not m: continue

        participant = _clean_label(m.group("participant"))
        run_id = int(m.group("run"))

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except: continue

        for role, metrics in METRICS_CONFIG.items():
            entries = doc.get(role, [])
            if not isinstance(entries, list): continue

            for (metric_name, dur_prefix, start_key, end_key) in metrics:
                for entry in entries:
                    val_ms = None

                    # 1. Try Native Nanoseconds first (Highest Precision)
                    val_ns = _safe_get(entry, f"{dur_prefix}_ns")
                    if val_ns is not None:
                        val_ms = val_ns / 1_000_000.0
                    else:
                        # 2. Fallback to pre-calculated High-Precision MS
                        val_ms = _safe_get(entry, f"{dur_prefix}_ms")

                        # 3. Fallback to Wall-Clock Timestamps subtraction
                        if val_ms is None:
                            t_s = _safe_get(entry, start_key)
                            t_e = _safe_get(entry, end_key)
                            if t_s is not None and t_e is not None:
                                val_ms = t_e - t_s

                    if val_ms is not None and val_ms >= 0:
                        data_points.append({
                            "participant": participant,
                            "run": run_id,
                            "role": role,
                            "metric": metric_name,
                            "duration_s": val_ms / 1000.0  # Kept in seconds for your existing downstream plots
                        })

    if not data_points:
        print("[WARN] No data found.")
        return

    # --- SAVE RAW DATA ---
    df = pd.DataFrame(data_points)
    raw_path = out_dir / RAW_FILE
    df.to_csv(raw_path, index=False)
    print(f"[OK] Raw data saved to: {raw_path}")

    # --- SAVE SUMMARY (With Percentiles) ---
    summary = df.groupby(["participant", "role", "metric"])["duration_s"].agg(
        count="count",
        mean_s="mean",
        std_s="std",
        min_s="min",
        max_s="max",
        median_s="median",
        p25_s=lambda x: x.quantile(0.25),
        p75_s=lambda x: x.quantile(0.75)
    ).reset_index()

    sum_path = out_dir / SUMMARY_FILE
    summary.to_csv(sum_path, index=False)
    print(f"[OK] Summary saved to: {sum_path}")

if __name__ == "__main__":
    main()