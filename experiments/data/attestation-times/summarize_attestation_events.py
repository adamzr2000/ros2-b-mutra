#!/usr/bin/env python3
"""
summarize_attestation_events.py

Fix: Use a shared per-run reference time run_t0 = min(t_start across all participants in that run).
This prevents timeline mismatch between Robot logs and SECaaS logs.
"""

import argparse
import json
from pathlib import Path
import re
from typing import Dict, Any, List, Optional, Tuple, DefaultDict
from collections import defaultdict
import pandas as pd

ROBOT_RE = re.compile(r"^(?P<participant>robot\d+)-run(?P<run>\d+)\.json$", re.IGNORECASE)
SECAAS_RE = re.compile(r"^(?P<participant>secaas)(?:-run(?P<run>\d+))?\.json$", re.IGNORECASE)

T_PREFIX = "t_"

def parse_args():
    p = argparse.ArgumentParser(description="Summarize attestation events (relative timestamps) into two JSONs.")
    p.add_argument("--base-dir", default=".", help="Base directory (default: current).")
    p.add_argument("--input-dir", default="results", help="Subdirectory with JSON inputs (default: test).")
    p.add_argument("--out-dir", default="_summary", help="Output directory (default: _summary).")
    p.add_argument("--N", type=int, default=8, help="First N prover attestations per Robot per run to include (default: 5).")
    return p.parse_args()

def _to_seconds_factor(time_unit: Optional[str]) -> float:
    if not time_unit:
        return 1.0 / 1000.0
    u = str(time_unit).strip().lower()
    if u in ("ms", "millisecond", "milliseconds"):
        return 1.0 / 1000.0
    if u in ("s", "sec", "second", "seconds"):
        return 1.0
    return 1.0 / 1000.0

def _num(x: Any) -> Optional[float]:
    try:
        return float(x)
    except Exception:
        return None

def _pretty_participant(name: str) -> str:
    n = name.strip().lower()
    if n.startswith("robot"):
        return "Robot" + n.replace("robot", "")
    if n == "secaas":
        return "SECaaS"
    return name.title()

def _order_key_robot(label: str) -> Tuple[int, int]:
    m = re.match(r"Robot(\d+)$", label)
    if m:
        return (1, int(m.group(1)))
    return (0, 0) if label == "SECaaS" else (2, 0)

def _order_key_att(att_label: str):
    m = re.match(r"att(\d+)-(Robot(\d+)|SECaaS)$", att_label)
    if not m:
        return (9999, 9999, 9999)
    k = int(m.group(1))
    who = m.group(2)
    if who == "SECaaS":
        return (k, 0, 0)
    mm = re.match(r"Robot(\d+)$", who)
    idx = int(mm.group(1)) if mm else 9999
    return (k, 1, idx)

def _collect_files(in_dir: Path) -> List[Tuple[str, int, Path]]:
    files: List[Tuple[str, int, Path]] = []
    for p in sorted(in_dir.rglob("*.json")):
        m = ROBOT_RE.match(p.name)
        if m:
            participant = m.group("participant").lower()
            run_id = int(m.group("run"))
            files.append((participant, run_id, p))
            continue
        s = SECAAS_RE.match(p.name)
        if s:
            participant = s.group("participant").lower()
            run = s.group("run")
            run_id = int(run) if run else 1
            files.append((participant, run_id, p))
            continue
    return files

def _safe_role_entries(data: Dict[str, Any], role: str) -> List[Dict[str, Any]]:
    v = data.get(role, [])
    return v if isinstance(v, list) else []

def _relative_timestamps(entry: Dict[str, Any], t0_abs: float, factor: float) -> Dict[str, float]:
    """
    Convert absolute timestamps to relative seconds using a shared per-run t0_abs.
    """
    out: Dict[str, float] = {}
    for k, v in entry.items():
        if isinstance(k, str) and k.startswith(T_PREFIX):
            tv = _num(v)
            if tv is not None:
                out[k] = (tv - t0_abs) * factor
    return out

def main():
    args = parse_args()
    base = Path(args.base_dir).resolve()
    in_dir = (base / args.input_dir).resolve()
    out_dir = (base / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not in_dir.exists():
        raise SystemExit(f"Input directory not found: {in_dir}")

    files = _collect_files(in_dir)
    if not files:
        raise SystemExit(f"No input JSON files found under {in_dir}")

    # Cache parsed inputs
    parsed_cache: Dict[Tuple[str, int], Dict[str, Any]] = {}
    for participant, run_id, path in files:
        with open(path, "r", encoding="utf-8") as f:
            parsed_cache[(participant, run_id)] = json.load(f)

    # -------------------- NEW: compute shared per-run reference t0 --------------------
    # run_t0_abs[run_id] = min(t_start across all participants in that run), in that file's native unit
    # But files can have different units, so we keep absolute in native and compare after converting to seconds.
    run_t0_seconds: Dict[int, float] = {}
    run_t0_abs_native: Dict[int, Tuple[float, float]] = {}
    # store (t0_abs_native_in_file_unit, factor) so we can use (t0_sec/factor) later if needed

    # First, compute earliest start in seconds for each run.
    starts_by_run: DefaultDict[int, List[float]] = defaultdict(list)
    # Also keep all (t_start_abs, factor) candidates for picking the absolute one consistently
    starts_native_by_run: DefaultDict[int, List[Tuple[float, float]]] = defaultdict(list)

    for (participant, run_id), data in parsed_cache.items():
        t_start = _num(data.get("t_start"))
        if t_start is None:
            print(f"[WARN] Missing t_start in {participant}-run{run_id}; will not contribute to run_t0.")
            continue
        factor = _to_seconds_factor(data.get("time_unit", "ms"))
        starts_by_run[run_id].append(t_start * factor)
        starts_native_by_run[run_id].append((t_start, factor))

    for run_id, starts_sec in starts_by_run.items():
        if not starts_sec:
            continue
        t0_sec = min(starts_sec)
        run_t0_seconds[run_id] = t0_sec
        # choose a native representation consistent with some factor: pick the entry that achieved min seconds
        best = min(starts_native_by_run[run_id], key=lambda tf: tf[0] * tf[1])
        run_t0_abs_native[run_id] = best  # (t0_abs, factor)

    if not run_t0_seconds:
        raise SystemExit("[ERROR] Could not compute run_t0 for any run (missing t_start everywhere).")

    # -------------------- First pass: determine first N prover ids per robot/run --------------------
    first_pass_index: Dict[Tuple[str, int], List[Tuple[float, str]]] = defaultdict(list)

    for (participant, run_id), data in parsed_cache.items():
        factor = _to_seconds_factor(data.get("time_unit", "ms"))
        # shared run t0 in ABS native units for this file:
        t0_sec = run_t0_seconds.get(run_id)
        if t0_sec is None:
            continue
        t0_abs_for_file = t0_sec / factor  # convert shared seconds back to this file's units

        if participant.startswith("robot"):
            for e in _safe_role_entries(data, "prover"):
                att_id = e.get("attestation_id")
                if not isinstance(att_id, str):
                    continue
                ts = _num(e.get("t_prover_start"))
                
                # Fallback for legacy logs or if key is missing
                if ts is None:
                    ts = _num(e.get("t_evidence_sent"))

                if ts is None:
                    t_keys = [k for k in e.keys() if isinstance(k, str) and k.startswith(T_PREFIX)]
                    ts_list = [_num(e[k]) for k in t_keys if _num(e[k]) is not None]
                    if not ts_list:
                        continue
                    ts = min(ts_list)

                rel = (ts - t0_abs_for_file) * factor
                first_pass_index[(participant, run_id)].append((rel, att_id))

    selected_ids_map: Dict[Tuple[str, int], Dict[str, str]] = {}
    for (participant, run_id), pairs in first_pass_index.items():
        pairs.sort(key=lambda x: x[0])
        top = pairs[: args.N]
        mapping: Dict[str, str] = {}
        pretty = _pretty_participant(participant)
        for i, (_, raw_id) in enumerate(top, start=1):
            mapping[raw_id] = f"att{i}-{pretty}"
        selected_ids_map[(participant, run_id)] = mapping

    run_union_map: Dict[int, Dict[str, str]] = defaultdict(dict)
    for (participant, run_id), m in selected_ids_map.items():
        run_union_map[run_id].update(m)

    by_run: Dict[int, Dict[str, Dict[str, Dict[str, Dict[str, float]]]]] = defaultdict(
        lambda: defaultdict(lambda: defaultdict(dict))
    )
    per_att_values: DefaultDict[Tuple[str, str, str, str], List[float]] = defaultdict(list)

    # -------------------- Second pass: fill by_run + per_att_values using shared run_t0 --------------------
    for (participant, run_id), data in parsed_cache.items():
        factor = _to_seconds_factor(data.get("time_unit", "ms"))
        t0_sec = run_t0_seconds.get(run_id)
        if t0_sec is None:
            print(f"[WARN] Missing run_t0 for run{run_id}; skipping file {participant}-run{run_id}.")
            continue
        t0_abs_for_file = t0_sec / factor

        P = _pretty_participant(participant)

        if participant.startswith("robot"):
            roles = ["prover", "verifier"]
        elif participant == "secaas":
            roles = ["verifier", "oracle"]
        else:
            roles = ["prover", "verifier", "oracle"]

        for role in roles:
            if participant.startswith("robot"):
                id_map = selected_ids_map.get((participant, run_id), {}) if role == "prover" else run_union_map.get(run_id, {})
            else:
                id_map = run_union_map.get(run_id, {})

            entries = _safe_role_entries(data, role)
            if not entries:
                continue

            for e in entries:
                raw_id = e.get("attestation_id")
                if not isinstance(raw_id, str):
                    continue

                if id_map:
                    new_id = id_map.get(raw_id)
                    if not new_id:
                        continue
                    att_label = new_id
                else:
                    att_label = raw_id

                rel_ts = _relative_timestamps(e, t0_abs_for_file, factor)
                if not rel_ts:
                    continue

                by_run[run_id][P][role][att_label] = rel_ts

                for t_key, v in rel_ts.items():
                    if v is not None:
                        per_att_values[(P, role, att_label, t_key)].append(float(v))

    # ------ Build outputs ------
    by_run_sorted: Dict[str, Any] = {}
    for run_id in sorted(by_run.keys()):
        run_key = f"run{run_id}"
        by_run_sorted[run_key] = {}
        for P in sorted(by_run[run_id].keys(), key=_order_key_robot):
            by_run_sorted[run_key][P] = {}
            for role in sorted(by_run[run_id][P].keys()):
                labels_sorted = sorted(by_run[run_id][P][role].keys(), key=_order_key_att)
                by_run_sorted[run_key][P][role] = {lbl: by_run[run_id][P][role][lbl] for lbl in labels_sorted}

    summary: Dict[str, Dict[str, Dict[str, Dict[str, Dict[str, float]]]]] = {}
    participants_sorted = sorted({k[0] for k in per_att_values.keys()}, key=_order_key_robot)
    for P in participants_sorted:
        summary.setdefault(P, {})
        roles_here = sorted({k[1] for k in per_att_values.keys() if k[0] == P})
        for role in roles_here:
            summary[P].setdefault(role, {})
            att_labels = sorted({k[2] for k in per_att_values.keys() if k[0] == P and k[1] == role}, key=_order_key_att)
            for att_label in att_labels:
                summary[P][role].setdefault(att_label, {})
                t_keys = sorted({k[3] for k in per_att_values.keys() if k[0] == P and k[1] == role and k[2] == att_label})
                for t_key in t_keys:
                    arr = per_att_values.get((P, role, att_label, t_key), [])
                    s = pd.Series(arr, dtype=float)
                    mean_s = float(s.mean(skipna=True)) if not s.empty else None
                    std_s  = float(s.std(ddof=1, skipna=True)) if len(s.dropna()) > 1 else 0.0
                    n_runs = int(s.count())
                    summary[P][role][att_label][t_key] = {
                        "mean_s": mean_s,
                        "std_s": std_s,
                        "n_runs": n_runs,
                    }

    out_by_run = out_dir / "attestation_events_by_run.json"
    with open(out_by_run, "w", encoding="utf-8") as f:
        json.dump(
            {
                "meta": {
                    "selected_first_N_per_robot_per_run": args.N,
                    "time_unit": "s",
                    "reference": "run_t0 = min(t_start across all participants in the same run)",
                    "note": "Relative times computed as (t_* - run_t0) in seconds. "
                            "IDs mapped via first-N prover order per robot/run; "
                            "verifier/oracle entries filtered to selected ids per run."
                },
                "by_run": by_run_sorted,
            },
            f, indent=2
        )

    out_summary = out_dir / "attestation_events_summary_per_attestation.json"
    with open(out_summary, "w", encoding="utf-8") as f:
        json.dump(
            {
                "meta": {
                    "selected_first_N_per_robot_per_run": args.N,
                    "time_unit": "s",
                    "reference": "run_t0 = min(t_start across all participants in the same run)",
                    "note": "Per-attestation stats across runs (simple pooling of values).",
                },
                "summary": summary,
            },
            f, indent=2
        )

    print(f"[OK] Wrote by-run: {out_by_run}")
    print(f"[OK] Wrote summary: {out_summary}")

if __name__ == "__main__":
    main()
