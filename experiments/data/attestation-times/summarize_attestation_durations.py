#!/usr/bin/env python3
"""
summarize_attestation_durations.py
Adapted to the high-precision "Sync Pair" architecture (_ns and _ms floats).

Results are expected under:
    results/<N{n_robots}>/startup/<participant>-run<N>.json
    results/<N{n_robots}>/continuous/<contract>/<participant>-SSP{ssp}ms-ITERQ{q}-cpu{cpu}-run<N>.json

e.g.  results/N4/startup/robot1-run3.json
      results/N4/startup/secaas-run1.json
      results/N4/continuous/rr/robot1-SSP20000ms-ITERQ1-cpu0p4-run2.json

Outputs (written to _summary/):
  durations_per_run_startup.csv    — startup mode; columns: n_robots, participant, run, role, metric, run_mean_s, participant_group
  durations_per_run_rr.csv         — continuous, RR contract; adds: ssp_ms, iterq, cpu_limit
  durations_per_run_lv.csv         — continuous, LV contract (written only if data exists)

EXPORT_RAW=True also writes durations_raw_startup.csv / durations_raw_{contract}.csv.
"""

import json
from pathlib import Path
import re
import pandas as pd

# --- Configuration ---
BASE_DIR  = Path(".")
INPUT_DIR = BASE_DIR / "results"
OUT_DIR   = BASE_DIR / "_summary"

EXPORT_RAW     = False  # one row per cycle — large, useful for debugging outliers
EXPORT_SUMMARY = False  # human-readable across-run summary — not used by plots

# Metric definitions — (metric_name, dur_prefix, start_key, end_key)
# dur_prefix is used to look up {dur_prefix}_ns or {dur_prefix}_ms first,
# then falls back to wall-clock subtraction (end_key - start_key).
# NOTE: dur_prover_e2e starts at p_send_evidence_START (same as evidence_call),
# so evidence_call ⊂ e2e_blockchain. Non-overlapping partition: local = total - e2e.
METRICS_CONFIG = {
    "prover": [
        ("e2e_blockchain",      "dur_prover_e2e",              "t_evidence_sent",                         "t_result_received"),
        ("evidence_call",       "dur_send_evidence_call",       "t_prover_start",                          "t_evidence_sent"),
        ("evidence_tx_confirm", "dur_send_evidence_tx_confirm", "t_evidence_sent",                         "t_evidence_sent_tx_confirmed"),
        ("total_lifecycle",     "dur_prover_total",             "t_prover_start",                          "t_prover_finished"),
    ],
    "verifier": [
        ("reaction_time",       "dur_verifier_reaction",        "t_ready_for_evaluation_received",         "t_verifier_start"),
        ("signatures_fetch",    "dur_signatures_fetch",         "t_get_signatures_start",                  "t_get_signatures_finished"),
        ("verify_compute",      "dur_verify_compute",           "t_verify_compute_start",                  "t_verify_compute_finished"),
        ("result_call",         "dur_send_result_call",         "t_verify_compute_finished",               "t_result_sent"),
        ("result_tx_confirm",   "dur_send_result_tx_confirm",   "t_result_sent",                           "t_result_sent_tx_confirmed"),
        ("total_lifecycle",     "dur_verifier_total",           "t_verifier_start",                        "t_verifier_finished"),
    ],
    "oracle": [
        ("reaction_time",            "dur_oracle_reaction",                     "t_attestation_started_received",          "t_oracle_start"),
        ("prover_addr_fetch",        "dur_prover_addr_fetch",                   "t_get_prover_addr_start",                 "t_get_prover_addr_finished"),
        ("db_lookup",                "dur_db_fetch",                            "t_get_prover_ref_signatures_db_start",    "t_get_prover_ref_signatures_db_finished"),
        ("ref_signature_call",       "dur_send_prover_ref_signature_call",      "t_get_prover_ref_signatures_db_finished", "t_prover_ref_signature_sent"),
        ("ref_signature_tx_confirm", "dur_send_prover_ref_signature_tx_confirm","t_prover_ref_signature_sent",             "t_prover_ref_signature_sent_tx_confirmed"),
        ("total_lifecycle",          "dur_oracle_total",                        "t_oracle_start",                          "t_oracle_finished"),
    ],
}

_FILE_PATTERN     = re.compile(r"^(?P<participant>.+)-run(?P<run>\d+)\.json$", re.IGNORECASE)
_KNOWN_MODES      = {"startup", "continuous"}
_KNOWN_CONTRACTS  = {"rr", "lv"}

# Param segment encoded in continuous filenames: -SSP{n}ms-ITERQ{n}-cpu{n}p{n}|NC
_PARAMS_RE = re.compile(
    r"-SSP(?P<ssp>\d+)ms-ITERQ(?P<iterq>\d+)-cpu(?P<cpu>[\dp]+|NC)$",
    re.IGNORECASE,
)

# Group keys per output file type
_STARTUP_KEYS = ["n_robots", "participant", "run", "role", "metric"]
_CONT_KEYS    = ["n_robots", "ssp_ms", "iterq", "cpu_limit", "participant", "run", "role", "metric"]


def _extract_params(base: str):
    """Strip -SSP/ITERQ/cpu suffix; return (ssp_ms, iterq, cpu_limit, cleaned_base).
    Returns (None, None, None, base) when absent (e.g. startup files)."""
    m = _PARAMS_RE.search(base)
    if m:
        ssp_ms  = int(m.group("ssp"))
        iterq   = int(m.group("iterq"))
        cpu_str = m.group("cpu")
        cpu     = None if cpu_str.upper() == "NC" else float(cpu_str.replace("p", ".").replace("P", "."))
        return ssp_ms, iterq, cpu, base[: m.start()]
    return None, None, None, base


def _parse_path(json_path: Path):
    """Return (n_robots, mode, contract, participant, ssp_ms, iterq, cpu_limit, run_id) or None."""
    parts = json_path.parts
    try:
        results_idx = next(i for i, p in enumerate(parts) if p == "results")
    except StopIteration:
        return None

    rel = parts[results_idx + 1:]

    # startup:    (N{n}, "startup", filename)           → 3 parts
    # continuous: (N{n}, "continuous", contract, filename) → 4 parts
    if len(rel) == 3:
        n_label, mode, filename = rel
        contract = ""
    elif len(rel) == 4:
        n_label, mode, contract, filename = rel
    else:
        return None

    if not n_label.upper().startswith("N"):
        return None
    if mode not in _KNOWN_MODES:
        return None
    if mode == "startup" and contract != "":
        return None
    if mode == "continuous" and contract not in _KNOWN_CONTRACTS:
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

    ssp_ms, iterq, cpu_limit, raw_participant = _extract_params(raw_participant)
    participant = _clean_label(raw_participant)
    return n_robots, mode, contract, participant, ssp_ms, iterq, cpu_limit, run_id


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


def _write_per_run(df: pd.DataFrame, group_keys: list, out_path: Path):
    """Average cycles within each run, add participant_group, write CSV."""
    per_run = (
        df.groupby(group_keys, as_index=False, dropna=False)["duration_s"]
        .mean()
        .rename(columns={"duration_s": "run_mean_s"})
    )
    per_run["participant_group"] = per_run["participant"].apply(_participant_group)
    per_run.to_csv(out_path, index=False)
    print(f"[OK] {out_path.name}")
    return per_run


def _write_summary(per_run: pd.DataFrame, group_keys: list, out_path: Path):
    agg = dict(
        run_count="count",
        mean_s="mean",
        std_s="std",
        min_s="min",
        max_s="max",
        median_s="median",
        p25_s=lambda x: x.quantile(0.25),
        p75_s=lambda x: x.quantile(0.75),
    )
    summary_keys = [k for k in group_keys if k not in ("run",)] + ["participant_group"]
    summary = (
        per_run.groupby(summary_keys, dropna=False)["run_mean_s"]
        .agg(**agg)
        .reset_index()
    )
    summary.to_csv(out_path, index=False)
    print(f"[OK] {out_path.name} (summary)")


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

        n_robots, mode, contract, participant, ssp_ms, iterq, cpu_limit, run_id = parsed

        try:
            with open(json_path, "r", encoding="utf-8") as f:
                doc = json.load(f)
        except Exception:
            continue

        for role, metrics in METRICS_CONFIG.items():
            entries = doc.get(role, [])
            if not isinstance(entries, list):
                continue
            for metric_name, dur_prefix, start_key, end_key in metrics:
                for entry in entries:
                    val_ms = None
                    val_ns = _safe_get(entry, f"{dur_prefix}_ns")
                    if val_ns is not None:
                        val_ms = val_ns / 1_000_000.0
                    else:
                        val_ms = _safe_get(entry, f"{dur_prefix}_ms")
                        if val_ms is None:
                            t_s = _safe_get(entry, start_key)
                            t_e = _safe_get(entry, end_key)
                            if t_s is not None and t_e is not None:
                                val_ms = t_e - t_s
                    if val_ms is not None and val_ms >= 0:
                        data_points.append({
                            "n_robots":   n_robots,
                            "mode":       mode,
                            "contract":   contract,
                            "ssp_ms":     ssp_ms,
                            "iterq":      iterq,
                            "cpu_limit":  cpu_limit,
                            "participant": participant,
                            "run":        run_id,
                            "role":       role,
                            "metric":     metric_name,
                            "duration_s": val_ms / 1000.0,
                        })

    if not data_points:
        print("[WARN] No data found.")
        return

    df = pd.DataFrame(data_points)

    # ── Startup ───────────────────────────────────────────────────────────────
    startup = df[df["mode"] == "startup"][
        _STARTUP_KEYS + ["duration_s"]
    ].copy()
    if not startup.empty:
        if EXPORT_RAW:
            startup.to_csv(out_dir / "durations_raw_startup.csv", index=False)
        pr = _write_per_run(startup, _STARTUP_KEYS, out_dir / "durations_per_run_startup.csv")
        if EXPORT_SUMMARY:
            _write_summary(pr, _STARTUP_KEYS, out_dir / "durations_summary_startup.csv")

    # ── Continuous — one file per contract ────────────────────────────────────
    cont = df[df["mode"] == "continuous"]
    for contract in sorted(cont["contract"].unique()):
        sub = cont[cont["contract"] == contract][
            _CONT_KEYS + ["duration_s"]
        ].copy()
        if EXPORT_RAW:
            sub.to_csv(out_dir / f"durations_raw_{contract}.csv", index=False)
        pr = _write_per_run(sub, _CONT_KEYS, out_dir / f"durations_per_run_{contract}.csv")
        if EXPORT_SUMMARY:
            _write_summary(pr, _CONT_KEYS, out_dir / f"durations_summary_{contract}.csv")


if __name__ == "__main__":
    main()
