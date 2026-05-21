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
import re

RESULTS_DIR     = os.path.join(os.path.dirname(__file__), "results")
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

# ── File-name helpers ─────────────────────────────────────────────────────────

_RUN_RE    = re.compile(r"-run(\d+)\.json$", re.IGNORECASE)
_PARAMS_RE = re.compile(r"SSP\d+ms-ITERQ\d+-cpu(?:[\dp]+|NC)", re.IGNORECASE)


def _listdir(directory):
    try:
        return os.listdir(directory)
    except FileNotFoundError:
        return []


def _file_tag(fname):
    """Extract param tag (e.g. 'SSP20000ms-ITERQ1-cpuNC') or '' for startup files."""
    m = _PARAMS_RE.search(fname)
    return m.group(0) if m else ""


def detect_param_tags(directory):
    """Return sorted unique param tags found in directory. Returns [''] if none (startup)."""
    tags = set()
    for f in _listdir(directory):
        if not f.lower().endswith(".json"):
            continue
        if not (f.lower().startswith("robot") or f.lower().startswith("secaas")):
            continue
        if not _RUN_RE.search(f):
            continue
        tags.add(_file_tag(f))
    return sorted(tags) or [""]


def detect_robots(directory, tag=""):
    """Return sorted robot indices for files matching the given param tag."""
    robots = set()
    for fname in _listdir(directory):
        if not fname.lower().startswith("robot"):
            continue
        if _file_tag(fname) != tag:
            continue
        m = re.match(r"^robot(\d+)", fname, re.IGNORECASE)
        if m:
            robots.add(int(m.group(1)))
    return sorted(robots)


def detect_runs(directory, tag=""):
    """Return sorted run numbers from secaas files matching the given param tag."""
    runs = set()
    for fname in _listdir(directory):
        if not fname.lower().startswith("secaas"):
            continue
        if _file_tag(fname) != tag:
            continue
        m = _RUN_RE.search(fname)
        if m:
            runs.add(int(m.group(1)))
    return sorted(runs)


def _robot_path(directory, ri, run, tag=""):
    if tag:
        return os.path.join(directory, f"robot{ri}-{tag}-run{run}.json")
    return os.path.join(directory, f"robot{ri}-run{run}.json")


def _secaas_path(directory, run, tag=""):
    if tag:
        return os.path.join(directory, f"secaas-{tag}-run{run}.json")
    return os.path.join(directory, f"secaas-run{run}.json")


# ── Helpers ───────────────────────────────────────────────────────────────────

def load_json(path):
    try:
        with open(path) as f:
            return json.load(f)
    except Exception:
        return None


def role_count(data, role):
    """Return len of a role list; handles both list and int (legacy)."""
    v = data.get(role, [])
    return len(v) if isinstance(v, list) else int(v)


def max_prover_duration(data):
    entries = data.get("prover", [])
    if not isinstance(entries, list) or not entries:
        return 0.0
    return max(e.get("dur_prover_e2e_ms", 0.0) for e in entries)


def _prover_total_ms(entry):
    """Extract total-lifecycle duration (ms) from a prover entry."""
    ns = entry.get("dur_prover_total_ns")
    if ns is not None:
        return float(ns) / 1_000_000.0
    ms = entry.get("dur_prover_total_ms")
    if ms is not None:
        return float(ms)
    t_s = entry.get("t_prover_start")
    t_e = entry.get("t_prover_finished")
    if t_s is not None and t_e is not None:
        return float(t_e) - float(t_s)
    return None


def prover_total_durations(data, label):
    """Return list of (duration_ms, label) for all prover total-lifecycle entries."""
    entries = data.get("prover", [])
    if not isinstance(entries, list):
        return []
    result = []
    for e in entries:
        d = _prover_total_ms(e)
        if d is not None and d >= 0:
            result.append((d, label))
    return result


def _report_cycle_stats(samples):
    """Print min/max total-lifecycle cycle time from (ms, label) samples."""
    if not samples:
        return
    mn_ms, mn_lbl = min(samples, key=lambda x: x[0])
    mx_ms, mx_lbl = max(samples, key=lambda x: x[0])
    info(f"cycle total_lifecycle — min: {mn_ms/1000:.3f}s ({mn_lbl})  "
         f"max: {mx_ms/1000:.3f}s ({mx_lbl})")


# ── Per-mode checks ───────────────────────────────────────────────────────────

def check_startup(directory, N, robots, runs):
    errors = warnings = 0
    all_samples = []

    for run in runs:
        secaas_data = load_json(_secaas_path(directory, run))
        secaas_oracle_count = 0

        if secaas_data is None:
            fail(f"run{run}: secaas file missing or unreadable")
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

        for ri in robots:
            rdata = load_json(_robot_path(directory, ri, run))

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
                warn(f"run{run} robot{ri}: prover dur={max_dur:.0f}ms "
                     f"(≥{TIMEOUT_WARN_MS}ms — possible timeout)")
                warnings += 1

            all_samples.extend(prover_total_durations(rdata, f"robot{ri}-run{run}"))

    _report_cycle_stats(all_samples)
    return errors, warnings


def check_continuous(directory, N, robots, runs, tag="", contract=""):
    errors = warnings = 0
    all_samples = []

    for run in runs:
        secaas_data = load_json(_secaas_path(directory, run, tag))
        robot_verifier_total = 0
        robot_prover_total   = 0

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
                if contract == "lv":
                    # LV design: the current root-of-trust robot falls back to SECaaS
                    # for its own attestation (can't self-verify). Expected ~1 per SSP
                    # cycle. Informational only.
                    info(f"run{run}: secaas elected verifier {verifier_count}× "
                         f"(LV self-verification fallback — expected)")
                else:
                    warn(f"run{run}: secaas was elected verifier {verifier_count}× "
                         f"(expected 0 in RR steady-state)")
                    warnings += 1

        for ri in robots:
            rdata = load_json(_robot_path(directory, ri, run, tag))

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
                warn(f"run{run} robot{ri}: max prover dur={max_dur:.0f}ms "
                     f"(≥{TIMEOUT_WARN_MS}ms — possible timeout)")
                warnings += 1

            robot_verifier_total += role_count(rdata, "verifier")
            all_samples.extend(prover_total_durations(rdata, f"robot{ri}-run{run}"))

        if robot_verifier_total == 0:
            fail(f"run{run}: total robot verifier entries=0 (no peer verification happened)")
            errors += 1

        oracle_n = role_count(secaas_data, "oracle") if secaas_data else 0
        timed_out = oracle_n - robot_prover_total if isinstance(oracle_n, int) else 0
        info(f"run{run}: prover_entries={robot_prover_total}  "
             f"peer_verifier_entries={robot_verifier_total}  "
             f"secaas_oracle={oracle_n}")
        if timed_out > 0:
            warn(f"run{run}: {timed_out} prover(s) timed out waiting for AttestationCompleted "
                 f"(oracle processed {oracle_n}, prover received {robot_prover_total})")
            warnings += 1

    _report_cycle_stats(all_samples)
    return errors, warnings


# ── Main ──────────────────────────────────────────────────────────────────────

def _print_header(label, N, n_robots, n_runs):
    print(f"\n{BOLD}{CYAN}{'─'*60}{RESET}")
    print(f"{BOLD}{CYAN}{label}  (N={N}, robots={n_robots}, runs={n_runs}){RESET}")
    print(f"{BOLD}{CYAN}{'─'*60}{RESET}")


def _report(e, w):
    if e == 0 and w == 0:
        ok("All checks passed")
    else:
        if e: fail(f"{e} error(s)")
        if w: warn(f"{w} warning(s)")


def main():
    results_dir = sys.argv[1] if len(sys.argv) > 1 else RESULTS_DIR

    n_dirs = sorted(
        d for d in (os.path.join(results_dir, e) for e in os.listdir(results_dir))
        if os.path.isdir(d) and re.match(r"N\d+$", os.path.basename(d))
    ) if os.path.isdir(results_dir) else []

    if not n_dirs:
        print(f"No N* directories found under {results_dir}")
        sys.exit(1)

    total_errors = total_warnings = 0

    for n_dir in n_dirs:
        n_label = os.path.basename(n_dir)
        N = int(re.search(r"(\d+)$", n_label).group(1))

        # ── Startup ───────────────────────────────────────────────────────────
        startup_dir = os.path.join(n_dir, "startup")
        if os.path.isdir(startup_dir):
            robots = detect_robots(startup_dir)
            runs   = detect_runs(startup_dir)
            if not robots or not runs:
                warn(f"{n_label}/startup: no data files found, skipping")
                total_warnings += 1
            else:
                _print_header(f"{n_label} / STARTUP", N, len(robots), len(runs))
                e, w = check_startup(startup_dir, N, robots, runs)
                total_errors += e; total_warnings += w
                _report(e, w)

        # ── Continuous ────────────────────────────────────────────────────────
        cont_dir = os.path.join(n_dir, "continuous")
        if not os.path.isdir(cont_dir):
            continue

        for contract in sorted(os.listdir(cont_dir)):
            contract_dir = os.path.join(cont_dir, contract)
            if not os.path.isdir(contract_dir):
                continue

            for tag in detect_param_tags(contract_dir):
                robots = detect_robots(contract_dir, tag)
                runs   = detect_runs(contract_dir, tag)
                tag_label = f"  [{tag}]" if tag else ""

                if not robots or not runs:
                    warn(f"{n_label}/continuous/{contract}{tag_label}: no data files found, skipping")
                    total_warnings += 1
                    continue

                _print_header(f"{n_label} / CONTINUOUS / {contract}{tag_label}",
                              N, len(robots), len(runs))
                e, w = check_continuous(contract_dir, N, robots, runs, tag, contract=contract)
                total_errors += e; total_warnings += w
                _report(e, w)

    print(f"\n{'═'*60}")
    if total_errors == 0 and total_warnings == 0:
        print(f"{BOLD}{GREEN}All checks passed across all N/mode combinations.{RESET}")
    elif total_errors:
        print(f"{BOLD}{RED}TOTAL: {total_errors} error(s)  {total_warnings} warning(s){RESET}")
    else:
        print(f"{BOLD}{YELLOW}TOTAL: 0 errors  {total_warnings} warning(s){RESET}")
    print()

    sys.exit(1 if total_errors else 0)


if __name__ == "__main__":
    main()
