#!/usr/bin/env python3
"""Robot impact experiment (contract V3): does the sidecar disturb the robot
application when it measures the robot's process memory at a high rate?

Setup: one Gazebo robot, sidecar without CPU cap; --ssp/--iterq size the
contract's freshness window for the fastest condition (10 ms x 500):
  ./start.sh --robots 1 --no-cpu-limit --ssp 10 --iterq 500

Measured: the arrivals of /robot1/tf, published at ~30 Hz by
robot_state_publisher, the attested process, as received by the topic
collector (docker-compose.topic-collector.yml, started and removed by this
script). A disturbance shows in the intervals between arrivals.

Conditions, in a new random order in each run:
  baseline  the sidecar does not attest
  SSP<ms>   the sidecar measures the robot's memory, waiting SSP ms between
            two measurements: at a low SSP it reads almost back to back. A
            V3 prover has one round open at a time (~3 s): K measurements
            are folded into each round (IterQ), K = 5 s / (SSP + the time of
            one measurement), so the memory is read at that rate while the
            chain keeps up. K is recorded, not varied.
Each condition attests for --settle seconds, is recorded for --window
seconds, then the sidecar stops and the host rests for --cooldown seconds
before the next one. Before the first run, one round is attested and not recorded: it
warms up the agents, checks that attestation succeeds and times one
measurement on this hardware.

Each condition of each run is written to
experiments/data/robot-impact/<condition>/runNN/:
  meta.json      setup, condition, K, window, memory reads and rounds in it
  arrivals.csv   one row per /tf message: t_recv_ns (monotonic clock of the
                 collector), ros_stamp_ns (Gazebo simulation clock)
  resources.csv  one row per second for robot1 and robot1-sidecar
Runs are appended (runNN numbering continues) only when the folder's runs
have the same setup. What a check finds is listed under "problems" in
meta.json. One experiment runs at a time; Ctrl-C stops the sidecar and the
condition in progress is not written.

Example:
  python3 experiment_robot_impact.py --runs 3 --window 60
"""

import argparse
import csv
import fcntl
import json
import os
import random
import subprocess
import sys
import time

import requests

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "robot-impact")
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
SIDECAR = "http://localhost:8001"  # robot1-sidecar
SECAAS = "http://localhost:8000"
TOPICS = "http://localhost:7001"   # topic collector
RESOURCES = "http://localhost:6000"
COLLECTOR_COMPOSE = os.path.join(ROOT, "docker-compose.topic-collector.yml")
CONTAINERS = ["robot1", "robot1-sidecar"]
BATCH_MS = 5000          # K x SSP: a batch lasts about one 5 s round
CYCLE_FLOOR_MS = 500     # the prover waits at least this long between batches
TF_HZ = 30               # robot_state_publisher's rate

ARRIVAL_COLUMNS = ["t_recv_ns", "ros_stamp_ns"]
RESOURCE_COLUMNS = ["t_ms", "container", "cpu_percent", "mem_mb", "net_rx_kb", "net_tx_kb"]
# meta.json keys that must be equal for all the runs of a condition folder.
SETUP_KEYS = ["contract", "dummy", "robots", "sidecar_cpu_limit", "gazebo_world", "block_period_s",
              "window_s", "settle_s", "cooldown_s", "iterq"]

HTTP = requests.Session()


def read_env():
    env = {}
    with open(os.path.join(ROOT, ".env")) as f:
        for line in f:
            key, sep, val = line.strip().partition("=")
            if sep and not key.startswith("#"):
                env[key.strip()] = val.strip()
    return env


def post(url, **kw):
    r = HTTP.post(url, timeout=kw.pop("timeout", 10), **kw)
    r.raise_for_status()
    return r.json()


def get(url, timeout=10):
    r = HTTP.get(url, timeout=timeout)
    r.raise_for_status()
    return r.json()


def iterq_for(ssp_ms, read_ms):
    """Measurements per round: a batch of at most BATCH_MS, each measurement
    taking read_ms and followed by SSP."""
    return max(1, int(BATCH_MS // (ssp_ms + read_ms)))


def expected_reads(ssp_ms, k, read_ms, seconds):
    """Memory reads in seconds of attestation: K per cycle of K reads, K-1
    waits of SSP and the wait between batches (at least CYCLE_FLOOR_MS)."""
    cycle_ms = k * read_ms + (k - 1) * ssp_ms + max(ssp_ms, CYCLE_FLOOR_MS)
    return seconds * 1000 * k / cycle_ms


def counters():
    return get(f"{SIDECAR}/status")["counters"]


def compose(*args):
    out = subprocess.run(["docker", "compose", "-f", COLLECTOR_COMPOSE, *args], cwd=ROOT,
                         capture_output=True, text=True, env={**os.environ, "COMPOSE_IGNORE_ORPHANS": "1"})
    if out.returncode != 0:
        sys.exit(f"docker compose {' '.join(args)} failed:\n{out.stderr}")


def start_topic_collector():
    """Starts the collector and waits until /robot1/tf arrives."""
    compose("up", "-d", "topic-collector")
    deadline = time.time() + 90
    seen = None
    while time.time() < deadline:
        try:
            n = get(f"{TOPICS}/status", timeout=3)["messages"]
            if seen is not None and n > seen:
                return
            seen = n
        except requests.RequestException:
            pass
        time.sleep(2)
    sys.exit("no /robot1/tf message at the topic collector: is the Gazebo robot running?")


def stop_sidecar():
    post(f"{SIDECAR}/stop", timeout=60)


def stop_all():
    for url in (f"{TOPICS}/stop", f"{RESOURCES}/stop"):
        try:
            post(url)
        except requests.RequestException:
            pass
    stop_sidecar()


# ── one condition ─────────────────────────────────────────────────────────────

def condition_dir(name):
    return os.path.join(OUT_ROOT, name)


def check_condition(base, setup):
    """Exits if the runs already in base were made with another setup."""
    if not os.path.isdir(base):
        return
    for d in sorted(os.listdir(base)):
        meta_path = os.path.join(base, d, "meta.json")
        if os.path.isfile(meta_path):
            with open(meta_path) as f:
                meta = json.load(f)
            diff = {k: (meta.get(k), setup[k]) for k in SETUP_KEYS if meta.get(k) != setup[k]}
            if diff:
                sys.exit(f"{os.path.relpath(base, ROOT)} holds runs of another setup ({d}: "
                         + ", ".join(f"{k} {old} → {new}" for k, (old, new) in diff.items())
                         + "): move them away or restore the setup")
            return


def write_run(base, files, meta):
    """Writes the run to the next runNN; a run is only named once complete."""
    os.makedirs(base, exist_ok=True)
    taken = [int(d[3:]) for d in os.listdir(base) if d.startswith("run") and d[3:].isdigit()]
    name = f"run{max(taken, default=0) + 1:02d}"
    tmp = os.path.join(base, f".{name}.tmp")
    os.makedirs(tmp)
    for fname, (columns, rows) in files.items():
        with open(os.path.join(tmp, fname), "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=columns, extrasaction="ignore")
            w.writeheader()
            w.writerows(rows)
    with open(os.path.join(tmp, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    os.rename(tmp, os.path.join(base, name))
    return os.path.join(base, name)


def run_condition(name, ssp_ms, args, setup, read_ms):
    k = iterq_for(ssp_ms, read_ms) if ssp_ms else None
    if ssp_ms:
        post(f"{SIDECAR}/config", json={"rounds": 0, "start_offset_ms": 0, "attestation_interval_ms": ssp_ms,
                                        "attestation_interval_mode": "fixed", "iterq_threshold": k})
        HTTP.delete(f"{SIDECAR}/results", timeout=10).raise_for_status()
        post(f"{SIDECAR}/start")
    time.sleep(args.settle)

    status = get(f"{SIDECAR}/status")
    before = status["counters"]
    post(f"{RESOURCES}/start", json={"containers": CONTAINERS})
    post(f"{TOPICS}/start")
    t_start = time.time()
    t0 = time.monotonic()
    time.sleep(args.window)
    post(f"{TOPICS}/stop")
    window_s = time.monotonic() - t0
    post(f"{RESOURCES}/stop")
    after = counters()
    if ssp_ms:
        stop_sidecar()
    time.sleep(args.cooldown)

    arrivals = get(f"{TOPICS}/results", timeout=30)["arrivals"]
    resources = get(f"{RESOURCES}/results", timeout=30)["samples"]
    delta = {c: after.get(c, 0) - before.get(c, 0) for c in after}

    problems = []
    gaps = [(b["t_recv_ns"] - a["t_recv_ns"]) / 1e6 for a, b in zip(arrivals, arrivals[1:])]
    if len(arrivals) < 0.5 * TF_HZ * window_s:
        problems.append(f"{len(arrivals)} /tf arrivals in {window_s:.0f} s, expected about {TF_HZ * window_s:.0f}")
    if gaps and max(gaps) > 1000:
        problems.append(f"/tf silent for {max(gaps):.0f} ms")
    if {r["container"] for r in resources} != set(CONTAINERS):
        problems.append(f"resource samples for {sorted({r['container'] for r in resources})}")
    if ssp_ms:
        expected = expected_reads(ssp_ms, k, read_ms, window_s)
        if delta["measurements"] < 0.7 * expected - 1:  # a window holds whole reads
            problems.append(f"{delta['measurements']} memory reads, expected about {expected:.0f}")
        bad = {c: delta[c] for c in ("failure", "stale", "timeouts", "errors") if delta[c]}
        if bad:
            problems.append(f"attestation {bad}")
    elif delta["measurements"] or status["status"] == "running":
        problems.append(f"the sidecar attested during the baseline ({delta['measurements']} memory reads)")

    print(f"   {name:9s} {len(arrivals)} arrivals ({len(arrivals) / window_s:.1f}/s), "
          f"{delta['measurements']} memory reads ({delta['measurements'] / window_s:.1f}/s), "
          f"{delta['success']} rounds, {len(problems)} problem(s)")
    for p in problems:
        print(f"   ⚠️  {p}")

    meta = {
        **setup, "condition": name, "ssp_ms": ssp_ms, "measurements_per_round": k,
        "measurement_ms": read_ms,
        "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_start)),
        "window_measured_s": round(window_s, 3), "arrivals": len(arrivals),
        "memory_reads": delta["measurements"], "memory_reads_per_s": round(delta["measurements"] / window_s, 2),
        "rounds_succeeded": delta["success"], "problems": problems,
        # gas estimates tried again after a Besu "Internal error" (+~100 ms each)
        "gas_estimate_retries": delta.get("gas_estimate_retries", 0),
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }
    write_run(condition_dir(name), {
        "arrivals.csv": (ARRIVAL_COLUMNS, arrivals),
        "resources.csv": (RESOURCE_COLUMNS, sorted(resources, key=lambda r: r["t_ms"])),
    }, meta)


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--runs", type=int, default=3, help="runs; each one records every condition (default: 3)")
    p.add_argument("--window", type=int, default=60, help="seconds recorded per condition (default: 60)")
    p.add_argument("--settle", type=int, default=10, help="seconds attested before recording (default: 10)")
    p.add_argument("--cooldown", type=int, default=5, help="seconds of rest after each condition (default: 5)")
    p.add_argument("--ssp", default="10000,5000,1000,100,10", help="SSPs in ms (default: 10000,5000,1000,100,10)")
    args = p.parse_args()
    ssps = [int(s) for s in args.ssp.split(",")]

    if env.get("CONTRACT") != "AttestationManagerV3":
        sys.exit("experiment_robot_impact.py measures contract V3: deploy it with ./start.sh --contract v3")
    if env.get("DUMMY") == "TRUE" or env.get("N_ROBOTS") != "1" or env.get("DEPLOY_MODE") == "remote":
        sys.exit("experiment_robot_impact.py needs one local Gazebo robot: "
                 "./start.sh --robots 1 --no-cpu-limit --ssp 10 --iterq 500")
    with open(os.path.join(ROOT, "blockchain/quorum-test-network/config/besu/genesis.json")) as f:
        block_period = json.load(f)["config"]["qbft"]["blockperiodseconds"]

    setup = {
        "contract": env["CONTRACT"], "dummy": False, "robots": 1, "sidecar_cpu_limit": env.get("CPU_LIMIT"),
        "gazebo_world": env.get("GAZEBO_WORLD"), "block_period_s": block_period,
        "window_s": args.window, "settle_s": args.settle, "cooldown_s": args.cooldown, "iterq": f"{BATCH_MS} ms / (SSP + measurement)",
    }
    conditions = [("baseline", None)] + [(f"SSP{s}", s) for s in ssps]
    for name, _ in conditions:
        check_condition(condition_dir(name), setup)

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")

    if "measurements" not in counters():
        sys.exit("the sidecar does not count its memory reads: rebuild it (make attestation-sidecar) "
                 "and ./start.sh again")
    print(f"setup: 1 Gazebo robot, sidecar CPU limit {setup['sidecar_cpu_limit']}, conditions "
          f"{', '.join(n for n, _ in conditions)}, {args.runs} run(s) of {args.window} s each")
    try:
        stop_sidecar()
        post(f"{SECAAS}/stop", timeout=120)  # a fresh SECaaS loop, kept running
        post(f"{SECAAS}/start")
        start_topic_collector()

        print("── warm-up: one round, not recorded")
        post(f"{SIDECAR}/config", json={"rounds": 1, "start_offset_ms": 0, "attestation_interval_ms": 10000,
                                        "attestation_interval_mode": "fixed", "iterq_threshold": 1})
        HTTP.delete(f"{SIDECAR}/results", timeout=10).raise_for_status()
        post(f"{SIDECAR}/start")
        deadline = time.time() + 60
        while get(f"{SIDECAR}/status").get("status") != "finished":
            if time.time() > deadline:
                sys.exit("the warm-up round did not succeed: check the sidecar's logs. If its image was "
                         "rebuilt, its references changed: run ./start.sh again")
            time.sleep(1)
        stop_sidecar()
        read_ms = round(get(f"{SIDECAR}/results")["prover"][0]["evidence_ms"], 1)
        print(f"   one measurement takes {read_ms} ms; K per SSP: "
              + ", ".join(f"{s} ms → {iterq_for(s, read_ms)}" for s in ssps))
        for ssp in ssps:
            k = iterq_for(ssp, read_ms)
            batch_ms = k * read_ms + (k - 1) * ssp
            needed = -(-2 * int(batch_ms) // (block_period * 1000)) + 15
            if needed > int(env["FRESHNESS_BLOCKS"]):
                sys.exit(f"SSP {ssp} ms with K={k} needs a freshness window of {needed} blocks, the contract has "
                         f"{env['FRESHNESS_BLOCKS']}: ./start.sh --robots 1 --no-cpu-limit --ssp 10 --iterq 500")

        for i in range(1, args.runs + 1):
            order = random.sample(conditions, len(conditions))
            print(f"── run {i}/{args.runs}: {', '.join(n for n, _ in order)}")
            for name, ssp in order:
                run_condition(name, ssp, args, setup, read_ms)
    except KeyboardInterrupt:
        print("\ninterrupted: the condition in progress is not written; stopping the sidecar")
        stop_all()
        sys.exit(130)
    finally:
        with open(os.path.join(ROOT, "experiments", "data", ".topic-collector.log"), "w") as f:
            subprocess.run(["docker", "logs", "topic-collector"], stdout=f, stderr=subprocess.STDOUT)
        compose("rm", "-sf", "topic-collector")


if __name__ == "__main__":
    main()
