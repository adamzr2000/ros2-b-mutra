#!/usr/bin/env python3

import json
from pathlib import Path
import re
from typing import Optional, List, Dict, Any
import pandas as pd

# --- Configuration ---
BASE_DIR = Path(".")
INPUT_DIR = BASE_DIR / "test"
OUT_DIR = BASE_DIR / "_summary"

# --- Output Filenames ---
PROVER_SUMMARY_FILE = "attestation_duration_per_participant.csv"
PROVER_RAW_FILE = "all_raw_attestation_durations.csv"

STEPS_SUMMARY_FILE = "attestation_steps_per_role.csv"
STEPS_RAW_FILE = "all_raw_attestation_steps.csv"

# --- Regex ---
# Captures generic participant names (robot1, secaas, etc.)
FILE_PATTERN = re.compile(r"^(?P<participant>.+)-run(?P<run>\d+)\.json$", re.IGNORECASE)

# --- Step Definitions (Snake Case) ---
# Format: (Step Name, Start Key, End Key)
STEP_DEFINITIONS = {
    "oracle": [
        ("blockchain_read", "t_get_prover_addr_start", "t_get_prover_addr_finished"),
        ("db_lookup", "t_ref_signatures_db_start", "t_ref_signatures_db_finished"),
        ("total_oracle", "t_attestation_started_received", "t_ref_signatures_sent"),
    ],
    "verifier": [
        ("blockchain_read", "t_get_signatures_start", "t_get_signatures_finished"),
        ("verification_logic", "t_get_signatures_finished", "t_result_sent"),
        ("total_verifier", "t_evaluation_ready_received", "t_result_sent"),
    ]
}

def _clean_label(raw: str) -> str:
    # Standardize names (e.g. "secaas" -> "SECaaS", "robot1" -> "Robot1")
    raw = raw.strip()
    if raw.lower() == "secaas":
        return "SECaaS"
    if raw.lower().startswith("robot"):
        return "Robot" + raw.lower().replace("robot", "")
    return raw.title()

def _unit_factor(time_unit: Optional[str]) -> float:
    if not time_unit:
        return 1.0 / 1000.0
    u = time_unit.strip().lower()
    if u in ("ms", "millisecond", "milliseconds"):
        return 1.0 / 1000.0
    if u in ("s", "sec", "second", "seconds"):
        return 1.0
    return 1.0 / 1000.0

def _safe_get(d: Dict[str, Any], key: str) -> Optional[float]:
    v = d.get(key, None)
    try:
        return float(v)
    except Exception:
        return None

def main():
    in_dir = INPUT_DIR.resolve()
    out_dir = OUT_DIR.resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not in_dir.exists():
        raise SystemExit(f"Input directory not found: {in_dir}")

    prover_per_run_rows: List[Dict[str, Any]] = []
    prover_raw_flat: List[Dict[str, Any]] = []
    steps_raw_flat: List[Dict[str, Any]] = []

    print(f"Reading JSONs from: {in_dir}")
    for json_path in sorted(in_dir.glob("*.json")):
        m = FILE_PATTERN.match(json_path.name)
        if not m:
            continue

        raw_name = m.group("participant")
        run_id = int(m.group("run"))
        participant_label = _clean_label(raw_name)

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                data = json.load(f)
        except Exception as e:
            print(f"[WARN] Skipping {json_path.name}: {e}")
            continue

        factor = _unit_factor(data.get("time_unit", "ms"))

        # --- A. Prover Cycle (End-to-End) ---
        prover_entries = data.get("prover", [])
        if isinstance(prover_entries, list) and prover_entries:
            durations_s: List[float] = []
            for idx, entry in enumerate(prover_entries):
                t_send = _safe_get(entry, "t_evidence_sent")
                t_recv = _safe_get(entry, "t_result_received")
                
                if t_send is not None and t_recv is not None:
                    delta_s = (t_recv - t_send) * factor
                    if delta_s >= 0:
                        durations_s.append(delta_s)
                        prover_raw_flat.append({
                            "participant": participant_label,
                            "duration_s": delta_s,
                            "run": run_id,
                            "att_index": idx
                        })
            
            if durations_s:
                per_run_mean_s = float(pd.Series(durations_s).mean())
                prover_per_run_rows.append({
                    "participant": participant_label,
                    "run": run_id,
                    "mean_duration_s": per_run_mean_s,
                    "n_attestations": len(durations_s),
                })

        # --- B. Internal Steps Breakdown ---
        for role, definitions in STEP_DEFINITIONS.items():
            entries = data.get(role, [])
            if not isinstance(entries, list) or not entries:
                continue
            
            for entry in entries:
                att_id = entry.get("attestation_id", "unknown")
                for (step_name, start_key, end_key) in definitions:
                    t_start = _safe_get(entry, start_key)
                    t_end = _safe_get(entry, end_key)

                    if t_start is not None and t_end is not None:
                        delta_s = (t_end - t_start) * factor
                        if delta_s >= 0:
                            steps_raw_flat.append({
                                "participant": participant_label,
                                "role": role.title(),
                                "step": step_name,
                                "duration_s": delta_s,
                                "run": run_id,
                                "attestation_id": att_id
                            })

    # --- Sorting Helper ---
    def _order_key(label: str):
        # 1. Robots, 2. SECaaS, 3. Others
        if label.lower().startswith("robot"):
            m = re.match(r"Robot(\d+)$", label)
            num = int(m.group(1)) if m else 999
            return (0, num)
        if label.lower() == "secaas":
            return (1, 0)
        return (2, label.lower())

    # =========================================================================
    # PART 1: PROVER OUTPUTS
    # =========================================================================
    if prover_per_run_rows:
        runs_df = pd.DataFrame(prover_per_run_rows)
        flat_df = pd.DataFrame(prover_raw_flat)

        bp_stats = flat_df.groupby("participant")["duration_s"].agg(
            median_s="median", p25_s=lambda x: x.quantile(0.25), p75_s=lambda x: x.quantile(0.75), min_s="min"
        ).reset_index()

        max_indices = flat_df.groupby("participant")["duration_s"].idxmax()
        max_rows = flat_df.loc[max_indices, ["participant", "duration_s", "run", "att_index"]]
        max_rows = max_rows.rename(columns={"duration_s": "max_s", "run": "max_s_run", "att_index": "max_s_index"})

        summary = (
            runs_df.groupby("participant", as_index=False)["mean_duration_s"]
            .agg(mean_of_means_s="mean", std_of_means_s="std")
            .merge(runs_df.groupby("participant", as_index=False)["run"].nunique().rename(columns={"run": "n_runs"}), on="participant")
            .merge(runs_df.groupby("participant", as_index=False)["n_attestations"].sum().rename(columns={"n_attestations": "total_attestations"}), on="participant")
        )

        summary = summary.merge(bp_stats, on="participant", how="left").merge(max_rows, on="participant", how="left")
        summary = summary.sort_values(by="participant", key=lambda s: s.map(_order_key)).reset_index(drop=True)

        cols = ["participant", "mean_of_means_s", "std_of_means_s", "median_s", "p25_s", "p75_s", "min_s", "max_s", "max_s_run", "max_s_index", "n_runs", "total_attestations"]
        summary = summary[cols]

        flat_df.sort_values(by=["participant", "run", "att_index"], key=lambda col: col if col.name != "participant" else col.map(_order_key)).to_csv(out_dir / PROVER_RAW_FILE, index=False)
        summary.to_csv(out_dir / PROVER_SUMMARY_FILE, index=False)
        print(f"[OK] Prover Raw Data: {out_dir / PROVER_RAW_FILE}")
        print(f"[OK] Prover Summary:  {out_dir / PROVER_SUMMARY_FILE}")
        
        print("\n--- Prover Summary Preview ---")
        print(summary.to_string(index=False))
    else:
        print("[WARN] No prover data found.")

    # =========================================================================
    # PART 2: STEP OUTPUTS
    # =========================================================================
    if steps_raw_flat:
        steps_df = pd.DataFrame(steps_raw_flat)
        per_run_steps = steps_df.groupby(["participant", "role", "step", "run"], as_index=False)["duration_s"].mean().rename(columns={"duration_s": "run_mean_s"})

        stats_mom = per_run_steps.groupby(["participant", "role", "step"], as_index=False)["run_mean_s"].agg(mean_of_means_s="mean", std_of_means_s="std")
        counts = steps_df.groupby(["participant", "role", "step"], as_index=False).agg(n_runs=("run", "nunique"), total_count=("duration_s", "count"))
        
        quantiles = steps_df.groupby(["participant", "role", "step"])["duration_s"].agg(
            median_s="median", p25_s=lambda x: x.quantile(0.25), p75_s=lambda x: x.quantile(0.75), min_s="min", max_s="max"
        ).reset_index()

        step_summary = stats_mom.merge(quantiles, on=["participant", "role", "step"]).merge(counts, on=["participant", "role", "step"])
        step_summary = step_summary.sort_values(by=["participant", "role", "step"], key=lambda col: col.map(_order_key) if col.name == "participant" else col)

        step_cols = ["participant", "role", "step", "mean_of_means_s", "std_of_means_s", "median_s", "p25_s", "p75_s", "min_s", "max_s", "n_runs", "total_count"]
        step_summary = step_summary[step_cols]

        steps_df.sort_values(by=["participant", "role", "step", "run"]).to_csv(out_dir / STEPS_RAW_FILE, index=False)
        step_summary.to_csv(out_dir / STEPS_SUMMARY_FILE, index=False)
        print(f"[OK] Steps Raw Data:  {out_dir / STEPS_RAW_FILE}")
        print(f"[OK] Steps Summary:   {out_dir / STEPS_SUMMARY_FILE}")
        
        print("\n--- Steps Summary Preview ---")
        print(step_summary.to_string(index=False))
    else:
        print("[WARN] No step data found.")

if __name__ == "__main__":
    main()