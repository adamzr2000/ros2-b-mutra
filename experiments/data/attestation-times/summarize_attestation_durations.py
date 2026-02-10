#!/usr/bin/env python3
"""
summarize_attestation_durations.py
Updates: Saves RAW data (for CDFs) AND full summary stats with quartiles (for Boxplots).
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

METRICS_CONFIG = {
    "prover": [
        ("e2e_blockchain", "dur_prover_e2e_ms", "t_evidence_sent", "t_result_received"),
        ("evidence_blockchain_write", "dur_send_evidence_tx_confirm_ms", "t_evidence_sent", "t_evidence_sent_tx_confirmed"),
        ("total_lifecycle", "dur_prover_total_ms", "t_prover_start", "t_prover_finished"),
    ],
    "verifier": [
        ("signatures_blockchain_read", "dur_signatures_fetch_ms", "t_get_signatures_start", "t_get_signatures_finished"),
        ("verify_compute", "dur_verify_compute_ms", "t_verify_compute_start", "t_verify_compute_finished"),
        ("result_blockchain_write", "dur_result_tx_ms", "t_result_sent", "t_verifier_finished"),
        ("total_lifecycle", "dur_verifier_total_ms", "t_verifier_start", "t_verifier_finished"),
    ],
    "oracle": [
        ("prover_credentials_blockchain_read", "dur_prover_addr_fetch_ms", "t_get_prover_addr_start", "t_get_prover_addr_finished"),
        ("db_lookup", "dur_oracle_db_fetch_ms", "t_get_prover_ref_signatures_db_start", "t_get_prover_ref_signatures_db_finished"),
        ("ref_signatures_blockchain_write", "dur_send_prover_ref_signatures_tx_confirm_ms", "t_prover_ref_signatures_sent", "t_oracle_finished"),
        ("total_lifecycle", "dur_oracle_total_ms", "t_oracle_start", "t_oracle_finished"),
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
    found_files = sorted(in_dir.glob("*.json"))
    
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
            
            for (metric_name, dur_key, start_key, end_key) in metrics:
                for entry in entries:
                    # 1. Try the standard MS key (e.g. dur_verify_compute_ms)
                    val_ms = _safe_get(entry, dur_key)

                    # 2. If result is 0 (precision loss) or None (missing), try finding a Microsecond key
                    if val_ms == 0 or val_ms is None:
                        # Construct a probable US key (e.g. dur_verify_compute_us)
                        # This works because your keys strictly follow the _ms / _us naming convention
                        us_key = dur_key.replace("_ms", "_us")
                        val_us = _safe_get(entry, us_key)
                        
                        if val_us is not None and val_us > 0:
                            # Convert microseconds to milliseconds
                            val_ms = val_us / 1000.0

                    # 3. If STILL None (or 0 and we want to try fallback), use Timestamps
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
                            "duration_s": val_ms / 1000.0
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
        p25_s=lambda x: x.quantile(0.25),  # <--- FIXED
        p75_s=lambda x: x.quantile(0.75)   # <--- FIXED
    ).reset_index()
    
    sum_path = out_dir / SUMMARY_FILE
    summary.to_csv(sum_path, index=False)
    print(f"[OK] Summary saved to: {sum_path}")

if __name__ == "__main__":
    main()