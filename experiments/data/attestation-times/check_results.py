#!/usr/bin/env python3
"""
check_results.py  —  Sanity-check all attestation-time result files.

Usage:
    python3 check_results.py [results_dir]

Checks per mode
───────────────
STARTUP (each robot does exactly 1 attestation per run)
  • Every robot file exists and is not empty
  • stopped == True
  • prover has exactly 1 entry  (robot completed its one attestation)
  • No prover entry has dur_prover_e2e_ms ≥ TIMEOUT_WARN_MS  (near-timeout)
  • SECaaS oracle count == N  (SECaaS saw every AttestationStarted event;
    a count < N indicates an attestation-ID collision where two robots generated
    the same ID and one overwrote the other in the contract)

  NOTE: we do NOT check secaas.verifier + Σrobot.verifier == N here.
  In startup mode, robot verifier exports are silently dropped when
  MarkExperimentStop has already been called (stopped=True) before the verifier
  goroutine finishes handling another robot's ReadyForEvaluation event.
  The authoritative success signal is prover=1 per robot.

CONTINUOUS (each robot does multiple attestations per run)
  • Every robot file exists and is not empty
  • stopped == True
  • prover has ≥ 1 entry
  • No prover entry has dur_prover_e2e_ms ≥ TIMEOUT_WARN_MS
  • Σ robot.verifier entries > 0 per run  (peer verification is happening)
  • SECaaS oracle count > 0 per run
  • SECaaS verifier count == 0 per run  (steady-state: robots do the work)
"""

import json
import os
import sys
import glob
import re
from collections import defaultdict

RESULTS_DIR   = os.path.join(os.path.dirname(__file__), "results")
TIMEOUT_WARN_MS = 30_000   # flag prover entries slower than this

# ── ANSI colours ──────────────────────────────────────────────────────────────
GREEN  = "\033[32m"
RED    = "\033[31m"
YELLOW = "\033[33m"
CYAN   = "\033[36m"
BOLD   = "\033[1m"
RESET  = "\033[0m"

def ok(msg):    print(f"  {GREEN}✔{RESET}  {msg}")
def fail(msg):  print(f"  {RED}✘{RESET}  {RED}{msg}{RESET}")
def warn(msg):  print(f"  {YELLOW}⚠{RESET}  {YELLOW}{msg}{RESET}")
def info(msg):  print(f"     {msg}")

# ── Helpers ───────────────────────────────────────────────────────────────────

def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception as e:
        return None

def detect_runs(directory, prefix):
    """Return the sorted list of run numbers found for the given file prefix."""
    pattern = os.path.join(directory, f"{prefix}-run*.json")
    nums = []
    for p in glob.glob(pattern):
        m = re.search(r"-run(\d+)\.json$", p)
        if m:
            nums.append(int(m.group(1)))
    return sorted(set(nums))

def detect_robots(directory, prefix):
    """Return sorted list of robot indices from files like prefix-robot{i}-run1.json."""
    pattern = os.path.join(directory, f"{prefix}-robot*-run1.json")
    indices = []
    for p in glob.glob(pattern):
        m = re.search(r"-robot(\d+)-run\d+\.json$", p)
        if m:
            indices.append(int(m.group(1)))
    return sorted(set(indices))

def role_count(data, role):
    """Return len of a role list; handles both list and int (legacy)."""
    v = data.get(role, [])
    return len(v) if isinstance(v, list) else int(v)

def max_prover_duration(data):
    entries = data.get("prover", [])
    if not isinstance(entries, list) or not entries:
        return 0.0
    return max(e.get("dur_prover_e2e_ms", 0.0) for e in entries)

# ── Per-mode checks ───────────────────────────────────────────────────────────

def check_startup(directory, N, robots, runs):
    errors = warnings = 0

    for run in runs:
        secaas_oracle_count = 0

        # ── SECaaS run file ──────────────────────────────────────────────────
        secaas_path = os.path.join(directory, f"secaas-run{run}.json")
        secaas_data = load_json(secaas_path)
        if secaas_data is None:
            fail(f"run{run}: secaas file missing or unreadable: {secaas_path}")
            errors += 1
        else:
            secaas_oracle_count = role_count(secaas_data, "oracle")
            if not secaas_data.get("stopped", False):
                fail(f"run{run}: secaas stopped=False")
                errors += 1
            if secaas_oracle_count != N:
                fail(f"run{run}: secaas oracle={secaas_oracle_count}, expected {N} "
                     f"(possible attestation-ID collision)")
                errors += 1

        # ── Robot files ──────────────────────────────────────────────────────
        for ri in robots:
            rpath = os.path.join(directory, f"startup-robot{ri}-run{run}.json")
            rdata = load_json(rpath)

            if rdata is None:
                fail(f"run{run} robot{ri}: file missing or unreadable")
                errors += 1
                continue

            if not rdata.get("stopped", False):
                fail(f"run{run} robot{ri}: stopped=False")
                errors += 1

            pc = role_count(rdata, "prover")
            if pc != 1:
                fail(f"run{run} robot{ri}: prover entries={pc}, expected 1")
                errors += 1

            max_dur = max_prover_duration(rdata)
            if max_dur >= TIMEOUT_WARN_MS:
                warn(f"run{run} robot{ri}: prover dur={max_dur:.0f}ms (≥{TIMEOUT_WARN_MS}ms — possible timeout)")
                warnings += 1

        # No "total resolved" check here — robot verifier exports are legitimately
        # dropped in startup mode when MarkExperimentStop is called before the
        # verifier goroutine finishes. prover=1 per robot is the success signal.

    return errors, warnings


def check_continuous(directory, N, robots, runs):
    errors = warnings = 0

    for run in runs:
        robot_verifier_total = 0
        robot_prover_total   = 0

        # ── SECaaS run file ──────────────────────────────────────────────────
        secaas_path = os.path.join(directory, f"secaas-run{run}.json")
        secaas_data = load_json(secaas_path)
        if secaas_data is None:
            fail(f"run{run}: secaas file missing or unreadable")
            errors += 1
        else:
            if not secaas_data.get("stopped", False):
                fail(f"run{run}: secaas stopped=False")
                errors += 1
            oracle_count = role_count(secaas_data, "oracle")
            if oracle_count == 0:
                fail(f"run{run}: secaas oracle=0 (no attestations processed)")
                errors += 1
            verifier_count = role_count(secaas_data, "verifier")
            if verifier_count > 0:
                warn(f"run{run}: secaas was elected verifier {verifier_count}× "
                     f"(expected 0 in continuous steady-state)")
                warnings += 1

        # ── Robot files ──────────────────────────────────────────────────────
        for ri in robots:
            rpath = os.path.join(directory, f"continuous-robot{ri}-run{run}.json")
            rdata = load_json(rpath)

            if rdata is None:
                fail(f"run{run} robot{ri}: file missing or unreadable")
                errors += 1
                continue

            if not rdata.get("stopped", False):
                fail(f"run{run} robot{ri}: stopped=False")
                errors += 1

            pc = role_count(rdata, "prover")
            if pc == 0:
                fail(f"run{run} robot{ri}: prover entries=0 (robot did no attestations)")
                errors += 1
            robot_prover_total += pc

            max_dur = max_prover_duration(rdata)
            if max_dur >= TIMEOUT_WARN_MS:
                warn(f"run{run} robot{ri}: max prover dur={max_dur:.0f}ms (≥{TIMEOUT_WARN_MS}ms — possible timeout)")
                warnings += 1

            robot_verifier_total += role_count(rdata, "verifier")

        # ── Coverage: peer verification must be happening ────────────────────
        if robot_verifier_total == 0:
            fail(f"run{run}: total robot verifier entries=0 (no peer verification happened)")
            errors += 1

        info(f"run{run}: prover_entries={robot_prover_total}  "
             f"peer_verifier_entries={robot_verifier_total}  "
             f"secaas_oracle={ role_count(secaas_data, 'oracle') if secaas_data else '?'}")

    return errors, warnings


# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else RESULTS_DIR

    n_dirs = sorted(
        d for d in glob.glob(os.path.join(results_dir, "N*"))
        if os.path.isdir(d)
    )
    if not n_dirs:
        print(f"No N* directories found under {results_dir}")
        sys.exit(1)

    total_errors = total_warnings = 0

    for n_dir in n_dirs:
        n_label = os.path.basename(n_dir)
        N = int(re.search(r"(\d+)$", n_label).group(1))

        for mode in ("startup", "continuous"):
            mode_dir = os.path.join(n_dir, mode)
            if not os.path.isdir(mode_dir):
                continue

            prefix  = mode
            robots  = detect_robots(mode_dir, prefix)
            runs    = detect_runs(mode_dir, "secaas")

            if not robots or not runs:
                warn(f"{n_label}/{mode}: no data files found, skipping")
                total_warnings += 1
                continue

            print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
            print(f"{BOLD}{CYAN}{n_label} / {mode.upper()}  "
                  f"(N={N}, robots={len(robots)}, runs={len(runs)}){RESET}")
            print(f"{BOLD}{CYAN}{'─'*60}{RESET}")

            if mode == "startup":
                e, w = check_startup(mode_dir, N, robots, runs)
            else:
                e, w = check_continuous(mode_dir, N, robots, runs)

            total_errors   += e
            total_warnings += w

            if e == 0 and w == 0:
                ok(f"All checks passed")
            else:
                if e:
                    fail(f"{e} error(s)")
                if w:
                    warn(f"{w} warning(s)")

    print(f"\n{'═'*60}")
    if total_errors == 0 and total_warnings == 0:
        print(f"{BOLD}{GREEN}All checks passed across all N/mode combinations.{RESET}")
    else:
        if total_errors:
            print(f"{BOLD}{RED}TOTAL: {total_errors} error(s)  {total_warnings} warning(s){RESET}")
        else:
            print(f"{BOLD}{YELLOW}TOTAL: 0 errors  {total_warnings} warning(s){RESET}")
    print()

    sys.exit(1 if total_errors else 0)


if __name__ == "__main__":
    main()
