#!/usr/bin/env python3
"""
Tamper detection latency benchmark.

Measures wall-clock time between in-memory binary tampering (via
tools/tamper.sh) and the first ❌ FAILURE attestation result logged
by the prover sidecar.

For each trial:
  1. Wait for a clean baseline (≥1 ✅ SUCCESS observed after recovery).
  2. Capture T0 (host clock) and run tamper.sh.
  3. Tail the prover sidecar's log; record T1 from the first
     "[Prover-...] Result: ❌ FAILURE" line with timestamp > T0.
  4. Recover the container(s) for the next trial.

Latency = T1 − T0. Lower bound is set by the prover's sleep until the
next measurement cycle (the SSP window) plus one block confirmation.

Prerequisites:
  - Stack started with --auto (continuous attestation) and ITERQ_THRESHOLD=1.
  - WAIT_FOR_VERIFICATION_RESULT=TRUE (default) so the prover sidecar
    logs the verifier's verdict; otherwise we can't read T1 from its log.
  - Attestation-sidecar image rebuilt with millisecond log timestamps
    (logger.go format "2006-01-02 15:04:05.000").

Usage:
  python3 run_experiments_and_collect_tamper_detection.py --ssp 20000 --trials 10
  python3 run_experiments_and_collect_tamper_detection.py --ssp 1000,5000,10000,20000 --trials 15
  python3 run_experiments_and_collect_tamper_detection.py --ssp 20000 --trials 10 --target sidecar
  python3 run_experiments_and_collect_tamper_detection.py --ssp 20000 --trials 10 --target both

Output:
  experiments/data/tamper-detection/results/<target>/SSP{N}ms-batch{B}.csv
  Columns: trial, t0_iso, t1_iso, latency_s, ssp_ms, target, robot
"""

import argparse
import csv
import os
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path

import requests

# ── Paths & endpoints ─────────────────────────────────────────────────────────

REPO_ROOT       = Path(__file__).parent.resolve()
TAMPER_SCRIPT   = REPO_ROOT / "tools" / "tamper.sh"
BASE_RESULTS    = REPO_ROOT / "experiments" / "data" / "tamper-detection" / "results"

ROBOTS_BASE_PORT = 8001   # robot1 = 8001, robot2 = 8002, ...

# Log line timestamp prefix produced by logger.go after the ms patch:
#   "2026-05-27 10:24:12.347 - INFO - ..."
LOG_TS_RE = re.compile(r"^(\d{4}-\d{2}-\d{2} \d{2}:\d{2}:\d{2}\.\d{3})\b")

# A FAILURE/SUCCESS line from the prover side specifically (not the verifier's "closed" line).
# Match plain ASCII words only — the ✅/❌ emoji are multi-byte UTF-8 that may be
# corrupted when docker logs are piped through a non-UTF-8 locale or cat -v.
PROVER_FAILURE_RE  = re.compile(r"\[Prover-.*?\].*FAILURE")
PROVER_SUCCESS_RE  = re.compile(r"\[Prover-.*?\].*SUCCESS")

# ── Small utilities ───────────────────────────────────────────────────────────

def iso_ms(ts: float) -> str:
    """UTC ISO-8601 with millisecond precision."""
    return datetime.fromtimestamp(ts, tz=timezone.utc).strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"


def parse_log_ts(line: str) -> float | None:
    """Extract the leading 'YYYY-MM-DD HH:MM:SS.mmm' from a sidecar log line.

    Docker containers default to UTC regardless of the host timezone, so
    logger.go's time.Now() produces UTC timestamps. We must interpret the
    parsed datetime as UTC (not local time) before converting to epoch —
    otherwise the comparison against time.time() (which is always UTC epoch)
    is off by the host's UTC offset.
    Returns a unix epoch float, or None if no match.
    """
    m = LOG_TS_RE.match(line)
    if not m:
        return None
    try:
        dt = datetime.strptime(m.group(1), "%Y-%m-%d %H:%M:%S.%f")
        return dt.replace(tzinfo=timezone.utc).timestamp()
    except ValueError:
        return None


def docker_logs_since(container: str, since_unix: float) -> str:
    """Fetch docker logs since the given unix timestamp (truncated to second)."""
    # `docker logs --since` accepts unix seconds. Subtract 1s for safety so we
    # don't lose a line whose log second equals T0's second (race condition).
    since = max(0, int(since_unix) - 1)
    try:
        out = subprocess.run(
            ["docker", "logs", "--since", str(since), container],
            capture_output=True, text=True,
            encoding="utf-8", errors="replace",  # safe decode regardless of locale
            timeout=10,
        )
        return out.stdout + out.stderr   # sidecar logs to stdout, but be safe
    except subprocess.TimeoutExpired:
        return ""


def container_running(name: str) -> bool:
    try:
        out = subprocess.run(
            ["docker", "ps", "--format", "{{.Names}}"],
            capture_output=True, text=True, timeout=5,
        ).stdout.splitlines()
        return name in out
    except subprocess.SubprocessError:
        return False


# ── HTTP config push ──────────────────────────────────────────────────────────

def stop_attestation(port: int) -> None:
    """POST /stop — idempotent, ignored if sidecar is already idle."""
    try:
        requests.post(f"http://localhost:{port}/stop", timeout=10)
    except requests.RequestException:
        pass


def start_attestation(port: int) -> None:
    """POST /start — begin (or resume) the attestation loop."""
    try:
        requests.post(f"http://localhost:{port}/start", timeout=10)
    except requests.RequestException:
        pass


def push_ssp(port: int, ssp_ms: int) -> None:
    """Set the prover's attestation interval.

    /config returns 409 while appState.measuring == true, so we stop the
    attestation loop first, push the config, then restart it.
    """
    stop_attestation(port)
    time.sleep(0.3)   # brief pause for the measuring flag to clear
    try:
        r = requests.post(
            f"http://localhost:{port}/config",
            json={"attestation_interval_ms": ssp_ms},
            timeout=5,
        )
        r.raise_for_status()
    except requests.RequestException as e:
        print(f"   ⚠️  /config push failed: {e}", file=sys.stderr)
    start_attestation(port)


def sidecar_alive(port: int, timeout: float = 30.0) -> bool:
    """Poll /status until the sidecar HTTP server responds."""
    deadline = time.time() + timeout
    while time.time() < deadline:
        try:
            requests.get(f"http://localhost:{port}/status", timeout=2)
            return True
        except requests.RequestException:
            time.sleep(0.5)
    return False


# ── Wait helpers ──────────────────────────────────────────────────────────────

def wait_for_prover_event(container: str, pattern: re.Pattern, since: float,
                          timeout: float, poll_s: float = 0.1) -> float | None:
    """Block until a log line matching `pattern` appears with parsed timestamp > since.

    Returns the parsed log timestamp (unix float) of the first match, or None on timeout.
    """
    deadline = time.time() + timeout
    while time.time() < deadline:
        logs = docker_logs_since(container, since)
        for line in logs.splitlines():
            ts = parse_log_ts(line)
            if ts is None or ts < since:
                continue
            if pattern.search(line):
                return ts
        time.sleep(poll_s)
    return None


def wait_for_clean_baseline(container: str, since: float, ssp_ms: int,
                            timeout_factor: float = 3.0) -> bool:
    """Wait for ≥1 ✅ SUCCESS attestation logged after `since`.

    Timeout = max(30s, ssp_ms * timeout_factor / 1000). At SSP=20s we wait ≤60s.
    """
    timeout = max(30.0, ssp_ms * timeout_factor / 1000.0)
    return wait_for_prover_event(container, PROVER_SUCCESS_RE, since, timeout) is not None


# ── Recovery ──────────────────────────────────────────────────────────────────

def recover(robot: str, sidecar: str, target: str) -> None:
    """Restore the container(s) corrupted by the tamper.

    state_publisher tamper → restart the robot container (kills sidecar via
    shared PID namespace), then start the sidecar.
    sidecar tamper         → restart only the sidecar.
    """
    if target == "sidecar":
        subprocess.run(["docker", "restart", sidecar], check=True,
                       stdout=subprocess.DEVNULL)
    else:
        subprocess.run(["docker", "restart", robot], check=True,
                       stdout=subprocess.DEVNULL)
        # robot1-sidecar exits with 137 when its PID namespace dies.
        # Brief sleep ensures Docker has fully processed the kill before we check.
        time.sleep(1)
        if not container_running(sidecar):
            subprocess.run(["docker", "start", sidecar], check=True,
                           stdout=subprocess.DEVNULL)


# ── Tamper invocation ─────────────────────────────────────────────────────────

def inject_tamper(robot: str, target: str) -> None:
    """Run tools/tamper.sh — exits immediately after writing the byte."""
    cmd = [str(TAMPER_SCRIPT), robot]
    if target != "state_publisher":
        cmd += ["--target", target]
    subprocess.run(cmd, check=True, stdout=subprocess.DEVNULL)


# ── CSV output ────────────────────────────────────────────────────────────────

def next_batch_csv(target: str, ssp_ms: int) -> Path:
    """Pick the next unused SSP{N}ms-batch{B}.csv name under results/<target>/."""
    out_dir = BASE_RESULTS / target
    out_dir.mkdir(parents=True, exist_ok=True)
    pattern = re.compile(rf"^SSP{ssp_ms}ms-batch(\d+)\.csv$")
    existing = [pattern.match(p.name) for p in out_dir.iterdir() if p.is_file()]
    used = [int(m.group(1)) for m in existing if m]
    nxt = (max(used) + 1) if used else 1
    return out_dir / f"SSP{ssp_ms}ms-batch{nxt}.csv"


def append_row(csv_path: Path, row: dict) -> None:
    is_new = not csv_path.exists()
    with csv_path.open("a", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(row.keys()))
        if is_new:
            w.writeheader()
        w.writerow(row)


# ── Trial loop ────────────────────────────────────────────────────────────────

def run_trial(trial_idx: int, robot: str, sidecar: str, sidecar_port: int,
              target: str, ssp_ms: int, csv_path: Path) -> bool:
    """One injection + detection + recovery cycle. Returns True if measured."""
    print(f"   [{trial_idx}] ", end="", flush=True)

    # 1. Baseline check — at least one ✅ SUCCESS since we entered the trial.
    baseline_since = time.time()
    print("baseline…", end=" ", flush=True)
    if not wait_for_clean_baseline(sidecar, baseline_since, ssp_ms):
        print("\n      ⚠️  no clean SUCCESS observed before tamper — skipping trial")
        return False

    # 2. Inject.
    # T0 is recorded immediately after inject_tamper() returns (not before).
    # The docker exec connection + dispatch overhead (~200–400 ms) is excluded,
    # so T0 is within ~20–50 ms of the actual byte write, giving a tighter
    # lower bound and reducing systematic latency overestimation.
    print("tamper…", end=" ", flush=True)
    try:
        inject_tamper(robot, target)
    except subprocess.CalledProcessError as e:
        print(f"\n      ❌ tamper.sh failed: {e}")
        return False
    t0 = time.time()
    t0_iso = iso_ms(t0)
    print(f"T0={t0_iso}", end=" ", flush=True)

    # 3. Wait for FAILURE in the prover log. Cap wait at 3×SSP + 10s.
    timeout = max(30.0, (ssp_ms / 1000.0) * 3.0 + 10.0)
    t1 = wait_for_prover_event(sidecar, PROVER_FAILURE_RE, t0, timeout)

    detected = t1 is not None
    latency  = round(t1 - t0, 3) if detected else None
    t1_iso   = iso_ms(t1) if detected else ""

    if detected:
        print(f"→ FAILURE@{t1_iso}  Δ={latency:.3f}s")
    else:
        print(f"\n      ❌ no FAILURE detected within {timeout:.0f}s")

    # 4. Persist — always write the row so detection_rate is computable:
    #    detection_rate = non-empty latency_s rows / total rows.
    append_row(csv_path, {
        "trial":     trial_idx,
        "t0_iso":    t0_iso,
        "t1_iso":    t1_iso,
        "latency_s": f"{latency:.3f}" if detected else "",
        "detected":  int(detected),
        "ssp_ms":    ssp_ms,
        "target":    target,
        "robot":     robot,
    })

    if not detected:
        recover(robot, sidecar, target)
        sidecar_alive(sidecar_port)
        push_ssp(sidecar_port, ssp_ms)   # container restart wipes env var SSP
        if target == "sidecar":
            time.sleep(min(ssp_ms / 1000.0, 5.0))   # flush pre-push goroutine
        return False

    # 5. Recover for the next trial.
    print("      🔄 recovering…", end=" ", flush=True)
    try:
        recover(robot, sidecar, target)
    except subprocess.CalledProcessError as e:
        print(f"\n      ❌ recovery failed: {e}")
        return False
    if not sidecar_alive(sidecar_port, timeout=45):
        print("\n      ❌ sidecar did not come back online")
        return False
    print("ok")

    # 6. Re-push SSP — config does not persist across container restarts.
    push_ssp(sidecar_port, ssp_ms)
    # For the sidecar target, the sidecar container auto-restarts with its
    # original ATTESTATION_INTERVAL_MS (SSP=20000ms from docker-compose).
    # It submits a goroutine immediately before push_ssp stops the loop.
    # That goroutine logs SUCCESS ~3.8s after restart (blockchain floor) —
    # BEFORE any new-SSP cycle goroutine exists — so the next trial's baseline
    # check would find it and inject too early in the correct SSP cycle,
    # inflating latency by ~(SSP_new - 2.3)s.  Sleeping ≥blockchain_floor here
    # ensures the stale goroutine's result is logged before baseline_since is
    # set, so the next trial only sees post-push successes.
    if target == "sidecar":
        time.sleep(min(ssp_ms / 1000.0, 5.0))
    return True


# ── Main ──────────────────────────────────────────────────────────────────────

def parse_ssps(s: str) -> list[int]:
    out = []
    for chunk in s.split(","):
        chunk = chunk.strip()
        if not chunk:
            continue
        try:
            out.append(int(chunk))
        except ValueError:
            raise argparse.ArgumentTypeError(f"invalid SSP value: {chunk!r}")
    if not out:
        raise argparse.ArgumentTypeError("--ssp requires at least one value")
    return out


def main():
    p = argparse.ArgumentParser(
        description="Tamper detection latency benchmark (B-MuTRA).",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    p.add_argument("--ssp", type=parse_ssps, required=True,
                   help="Attestation interval(s) in ms, comma-separated. e.g. 1000,5000,10000,20000")
    p.add_argument("--trials", type=int, default=10,
                   help="Number of injection trials per (target, SSP). Default: 10")
    p.add_argument("--target", choices=["state_publisher", "sidecar", "both"],
                   default="state_publisher",
                   help="Tamper target. 'both' runs state_publisher then sidecar.")
    p.add_argument("--robot", default="robot1",
                   help="Which robot to target. Default: robot1")
    args = p.parse_args()

    if not TAMPER_SCRIPT.exists():
        sys.exit(f"❌ tamper script not found: {TAMPER_SCRIPT}")

    sidecar = f"{args.robot}-sidecar"
    # Port mapping: robot1 → 8001, robot2 → 8002, ...
    try:
        idx = int(args.robot.removeprefix("robot"))
    except ValueError:
        sys.exit(f"❌ cannot infer port for {args.robot} (expected robot<N>)")
    sidecar_port = ROBOTS_BASE_PORT + (idx - 1)

    if not container_running(sidecar):
        sys.exit(f"❌ container {sidecar} is not running — start the stack first")
    if not sidecar_alive(sidecar_port, timeout=10):
        sys.exit(f"❌ sidecar {sidecar} not reachable on port {sidecar_port}")

    targets = ["state_publisher", "sidecar"] if args.target == "both" else [args.target]

    print(f"📋 robot={args.robot}  sidecar={sidecar}:{sidecar_port}")
    print(f"📋 targets={targets}  ssps={args.ssp}ms  trials={args.trials} each")
    print(f"📁 results → {BASE_RESULTS}/<target>/SSPNms-batchN.csv")

    total_attempted = 0
    total_succeeded = 0

    for target in targets:
        for ssp_ms in args.ssp:
            csv_path = next_batch_csv(target, ssp_ms)
            print(f"\n{'='*60}")
            print(f"🏁 target={target}  SSP={ssp_ms}ms")
            print(f"   output → {csv_path.relative_to(REPO_ROOT)}")
            print(f"{'='*60}")

            push_ssp(sidecar_port, ssp_ms)
            # Give the sidecar one cycle to switch to the new SSP.
            time.sleep(min(ssp_ms / 1000.0, 5.0))

            for i in range(1, args.trials + 1):
                total_attempted += 1
                if run_trial(i, args.robot, sidecar, sidecar_port,
                             target, ssp_ms, csv_path):
                    total_succeeded += 1

    print(f"\n🎉 Done. {total_succeeded}/{total_attempted} trials recorded.")
    print(f"   Results in {BASE_RESULTS}")


if __name__ == "__main__":
    main()
