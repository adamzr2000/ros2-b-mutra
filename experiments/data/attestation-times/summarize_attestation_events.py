#!/usr/bin/env python3
"""
summarize_attestation_events.py

Produces two JSON outputs from raw attestation logs:
  - attestation_events_by_run.json  : per-run relative timestamps, keyed by N+mode
  - attestation_events_summary.json : mean/std of relative timestamps across runs, keyed by N+mode

Timestamps are relative to run_t0 = min(t_start across all participants in that
N/mode/run group), converted to seconds.  They are intentionally kept as absolute
relative times (not durations) so that event-plot scripts can reconstruct full
per-attestation timelines directly.

Expected directory layout under --input-dir:
  N{n}/{mode}/{mode}-robot{i}-run{k}.json
  N{n}/{mode}/secaas-run{k}.json

where mode ∈ {continuous, startup}.
"""

import argparse
import json
from pathlib import Path
import re
from typing import Any, Dict, List, Optional, Tuple
from collections import defaultdict
import pandas as pd

ROBOT_RE  = re.compile(r"^(?:\w+-)?(?P<participant>robot\d+)-run(?P<run>\d+)\.json$", re.IGNORECASE)
SECAAS_RE = re.compile(r"^(?P<participant>secaas)(?:-run(?P<run>\d+))?\.json$", re.IGNORECASE)

T_PREFIX = "t_"
KNOWN_MODES = {"continuous", "startup"}


def parse_args():
    p = argparse.ArgumentParser(
        description="Summarize attestation event timestamps into JSON files for event plots."
    )
    p.add_argument("--base-dir",  default=".", help="Base directory (default: current).")
    p.add_argument("--input-dir", default="results",  help="Subdirectory with N{n}/{mode}/ layout (default: results).")
    p.add_argument("--out-dir",   default="_summary", help="Output directory (default: _summary).")
    p.add_argument("--N", type=int, default=8,
                   help="First N prover attestations per robot per run to include (default: 8).")
    return p.parse_args()


# ── helpers ────────────────────────────────────────────────────────────────────

def _to_seconds_factor(time_unit: Optional[str]) -> float:
    if time_unit and str(time_unit).strip().lower() in ("s", "sec", "second", "seconds"):
        return 1.0
    return 1.0 / 1000.0  # default: treat as ms


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
    k   = int(m.group(1))
    who = m.group(2)
    if who == "SECaaS":
        return (k, 0, 0)
    mm  = re.match(r"Robot(\d+)$", who)
    idx = int(mm.group(1)) if mm else 9999
    return (k, 1, idx)


def _safe_role_entries(data: Dict[str, Any], role: str) -> List[Dict[str, Any]]:
    v = data.get(role, [])
    return v if isinstance(v, list) else []


def _relative_timestamps(entry: Dict[str, Any], t0_native: float, factor: float) -> Dict[str, float]:
    """Convert absolute native-unit t_* fields to seconds relative to t0_native."""
    out = {}
    for k, v in entry.items():
        if isinstance(k, str) and k.startswith(T_PREFIX):
            tv = _num(v)
            if tv is not None:
                out[k] = (tv - t0_native) * factor
    return out


# ── file collection ────────────────────────────────────────────────────────────

def _collect_files(in_dir: Path) -> List[Tuple[Optional[int], Optional[str], str, int, Path]]:
    """
    Return list of (n_robots, mode, participant, run_id, path).
    n_robots and mode are extracted from the N{n}/{mode}/ directory structure;
    they are None when the layout does not match (files processed but not grouped).
    """
    results = []
    for p in sorted(in_dir.rglob("*.json")):
        parts = p.relative_to(in_dir).parts  # e.g. ('N4', 'continuous', 'continuous-robot1-run1.json')

        n_robots: Optional[int] = None
        mode:     Optional[str] = None
        if len(parts) >= 3:
            try:
                n_robots = int(parts[0].lstrip("Nn"))
            except ValueError:
                pass
            if parts[1].lower() in KNOWN_MODES:
                mode = parts[1].lower()

        m = ROBOT_RE.match(p.name)
        if m:
            results.append((n_robots, mode, m.group("participant").lower(), int(m.group("run")), p))
            continue
        s = SECAAS_RE.match(p.name)
        if s:
            run_str = s.group("run")
            results.append((n_robots, mode, s.group("participant").lower(), int(run_str) if run_str else 1, p))
    return results


# ── per-group processing ───────────────────────────────────────────────────────

def _process_group(
    group_files: List[Tuple[str, int, Path]],
    n_att: int,
) -> Tuple[Dict, Dict]:
    """
    Process one (n_robots, mode) group.
    group_files: list of (participant, run_id, path)
    Returns (by_run_sorted, summary).
    """
    # Load JSON
    parsed: Dict[Tuple[str, int], Dict[str, Any]] = {}
    for participant, run_id, path in group_files:
        with open(path, "r", encoding="utf-8") as f:
            parsed[(participant, run_id)] = json.load(f)

    # Shared per-run t0: minimum t_start (seconds) across all participants in that run
    starts_by_run: Dict[int, List[float]] = defaultdict(list)
    for (participant, run_id), data in parsed.items():
        t_start = _num(data.get("t_start"))
        if t_start is None:
            print(f"  [WARN] Missing t_start in {participant}-run{run_id}")
            continue
        factor = _to_seconds_factor(data.get("time_unit"))
        starts_by_run[run_id].append(t_start * factor)

    run_t0_sec: Dict[int, float] = {
        rid: min(starts) for rid, starts in starts_by_run.items() if starts
    }
    if not run_t0_sec:
        print("  [WARN] No t_start found in any file — skipping group.")
        return {}, {}

    # First pass: find first n_att prover attestation IDs per robot/run (sorted by t_prover_start)
    selected_ids: Dict[Tuple[str, int], Dict[str, str]] = {}
    for (participant, run_id), data in parsed.items():
        if not participant.startswith("robot"):
            continue
        t0_sec = run_t0_sec.get(run_id)
        if t0_sec is None:
            continue
        factor   = _to_seconds_factor(data.get("time_unit"))
        t0_native = t0_sec / factor

        pairs: List[Tuple[float, str]] = []
        for e in _safe_role_entries(data, "prover"):
            att_id = e.get("attestation_id")
            if not isinstance(att_id, str):
                continue
            ts = _num(e.get("t_prover_start")) or _num(e.get("t_evidence_sent"))
            if ts is None:
                t_vals = [_num(e[k]) for k in e if isinstance(k, str) and k.startswith(T_PREFIX)]
                t_vals = [v for v in t_vals if v is not None]
                if not t_vals:
                    continue
                ts = min(t_vals)
            pairs.append(((ts - t0_native) * factor, att_id))

        pairs.sort(key=lambda x: x[0])
        pretty = _pretty_participant(participant)
        selected_ids[(participant, run_id)] = {
            raw_id: f"att{i}-{pretty}"
            for i, (_, raw_id) in enumerate(pairs[:n_att], start=1)
        }

    # Union of selected IDs per run (used to filter verifier/oracle entries)
    run_union: Dict[int, Dict[str, str]] = defaultdict(dict)
    for (participant, run_id), mapping in selected_ids.items():
        run_union[run_id].update(mapping)

    # Second pass: collect relative timestamps
    by_run:   Dict[int, Any]                             = defaultdict(lambda: defaultdict(lambda: defaultdict(dict)))
    per_att:  Dict[Tuple[str, str, str, str], List[float]] = defaultdict(list)

    for (participant, run_id), data in parsed.items():
        t0_sec = run_t0_sec.get(run_id)
        if t0_sec is None:
            continue
        factor    = _to_seconds_factor(data.get("time_unit"))
        t0_native = t0_sec / factor
        P         = _pretty_participant(participant)

        roles = ["prover", "verifier"] if participant.startswith("robot") else ["verifier", "oracle"]

        for role in roles:
            id_map = (
                selected_ids.get((participant, run_id), {}) if role == "prover"
                else run_union.get(run_id, {})
            )
            for e in _safe_role_entries(data, role):
                raw_id = e.get("attestation_id")
                if not isinstance(raw_id, str):
                    continue
                att_label = id_map.get(raw_id)
                if not att_label:
                    continue
                rel = _relative_timestamps(e, t0_native, factor)
                if not rel:
                    continue
                by_run[run_id][P][role][att_label] = rel
                for t_key, v in rel.items():
                    per_att[(P, role, att_label, t_key)].append(v)

    # Sort by_run
    by_run_sorted: Dict[str, Any] = {}
    for run_id in sorted(by_run):
        rk = f"run{run_id}"
        by_run_sorted[rk] = {}
        for P in sorted(by_run[run_id], key=_order_key_robot):
            by_run_sorted[rk][P] = {}
            for role in sorted(by_run[run_id][P]):
                labels = sorted(by_run[run_id][P][role], key=_order_key_att)
                by_run_sorted[rk][P][role] = {lbl: by_run[run_id][P][role][lbl] for lbl in labels}

    # Build summary (mean/std of relative timestamps across runs)
    summary: Dict[str, Any] = {}
    for P in sorted({k[0] for k in per_att}, key=_order_key_robot):
        summary[P] = {}
        for role in sorted({k[1] for k in per_att if k[0] == P}):
            summary[P][role] = {}
            att_labels = sorted({k[2] for k in per_att if k[0] == P and k[1] == role}, key=_order_key_att)
            for att_label in att_labels:
                summary[P][role][att_label] = {}
                t_keys = sorted({k[3] for k in per_att if k[0] == P and k[1] == role and k[2] == att_label})
                for t_key in t_keys:
                    arr = per_att[(P, role, att_label, t_key)]
                    s   = pd.Series(arr, dtype=float)
                    summary[P][role][att_label][t_key] = {
                        "mean_s": float(s.mean()),
                        "std_s":  float(s.std(ddof=1)) if len(s) > 1 else 0.0,
                        "n_runs": int(s.count()),
                    }

    return by_run_sorted, summary


# ── entry point ────────────────────────────────────────────────────────────────

def main():
    args    = parse_args()
    base    = Path(args.base_dir).resolve()
    in_dir  = (base / args.input_dir).resolve()
    out_dir = (base / args.out_dir).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    if not in_dir.exists():
        raise SystemExit(f"Input directory not found: {in_dir}")

    all_files = _collect_files(in_dir)
    if not all_files:
        raise SystemExit(f"No JSON files found under {in_dir}")

    # Group by (n_robots, mode) — each combination is processed independently
    groups: Dict[Tuple[Optional[int], Optional[str]], List[Tuple[str, int, Path]]] = defaultdict(list)
    for n_robots, mode, participant, run_id, path in all_files:
        groups[(n_robots, mode)].append((participant, run_id, path))

    full_by_run: Dict[str, Any] = {}
    full_summary: Dict[str, Any] = {}

    for (n_robots, mode), group_files in sorted(
        groups.items(),
        key=lambda kv: (kv[0][0] or 0, kv[0][1] or ""),
    ):
        n_label = str(n_robots) if n_robots is not None else "unknown"
        m_label = mode if mode else "unknown"
        label   = f"N{n_label}_{m_label}"
        print(f"[{label}] {len(group_files)} files …")

        by_run, summary = _process_group(group_files, args.N)
        if by_run:
            full_by_run[label]  = by_run
            full_summary[label] = summary

    meta = {
        "selected_first_N_per_robot_per_run": args.N,
        "time_unit": "s",
        "keys": "N{n}_{mode}  →  run{k}  →  participant  →  role  →  att{i}-{Participant}  →  t_key  →  seconds",
        "reference": "run_t0 = min(t_start across all participants in the same N/mode/run)",
        "note": (
            "Timestamps are absolute relative times (t_event - run_t0) in seconds, "
            "not durations. Use by_run for per-run event plots; use summary for "
            "cross-run mean ± std overlays."
        ),
    }

    out_by_run = out_dir / "attestation_events_by_run.json"
    with open(out_by_run, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "data": full_by_run}, f, indent=2)
    print(f"\n[OK] Wrote by-run:  {out_by_run}")

    out_summary = out_dir / "attestation_events_summary.json"
    with open(out_summary, "w", encoding="utf-8") as f:
        json.dump({"meta": meta, "data": full_summary}, f, indent=2)
    print(f"[OK] Wrote summary: {out_summary}")


if __name__ == "__main__":
    main()
