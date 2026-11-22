#!/usr/bin/env python3
"""Tamper impact experiment (contract V3): what a tampered robot does to the
swarm's mission, and how detection and isolation recover it.

Four Gazebo robots move to a goal in a square formation (formation_agent,
the attested process). In attack runs, robot3's formation_agent is tampered
with in memory (tools/tamper.sh --target formation_agent: its steering turns
into a constant turn): robot3 spins and drags the formation. On the failure
verdict, the orchestrator (docker-compose.formation.yml) excludes robot3 when
mitigation is on, and the others reform a triangle and go on.

Scenarios, in a new random order in each repetition:
  baseline              no attack
  unmitigated-SSP<ms>   attack, failure only recorded (--unmitigated-ssp)
  mitigated-SSP<ms>     attack, robot3 excluded on failure (--mitigated-ssp)

Each run deploys a fresh stack (./start.sh --robots 4 --formation --auto),
so every mission starts from the same state, and tears it down after the
fixed window (which writes the bag). Attack runs tamper --attack-delay
seconds after the goal is published, right after a measurement of robot3
(worst case: detection one SSP plus one round later). The orchestrator
checks the chain every 0.2 s. The metrics are computed offline from the bag
by tools/formation_metrics.py.

Each run is written to experiments/data/tamper-impact/<scenario>/runNN/:
  meta.json       setup, scenario, attack and detection times, problems
  bag/            the recorded ros2 bag (ground truth, odometry, markers)
  events.json     the orchestrator's record (failure verdicts seen)
  inject.json     the tampering time
  formation.csv   per step: formation error, damage, progress, cross-track
  metrics.json    summary: progress, cumulative damage, formation error, ...
  start.log       the output of start.sh
Runs are appended (runNN numbering continues) only when the folder's runs
have the same setup. One experiment runs at a time; Ctrl-C tears the stack
down and the run in progress is not written.

Example (~11 min per run on this host, 12 runs):
  python3 experiment_tamper_impact.py --reps 3
"""

import argparse
import fcntl
import json
import os
import random
import re
import shutil
import subprocess
import sys
import time
from datetime import datetime, timezone

import requests

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "tamper-impact")
STAGING_SUBDIR = "tamper-impact/.staging"  # under experiments/data: where the stack writes a run
STAGING = os.path.join(ROOT, "experiments", "data", STAGING_SUBDIR)
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
VICTIM = "robot3"  # the formation slot tables assume robot3 is the victim
VICTIM_SIDECAR = "http://localhost:8003"
TAG = "run"
TAMPERED_RE = re.compile(r"tampered .* at (\d+) ms")
# meta.json keys that must be equal for all the runs of a scenario folder.
SETUP_KEYS = ["contract", "robots", "victim", "goal_x", "window_s", "attack_delay_s", "k_form", "k_att",
              "formation_scale", "poll_interval_s", "gpu", "sidecar_cpu_limit", "iterq", "jury_size",
              "jury_threshold"]


def log(msg):
    print(f"[{time.strftime('%H:%M:%S')}] {msg}", flush=True)


def read_env():
    env = {}
    with open(os.path.join(ROOT, ".env")) as f:
        for line in f:
            key, sep, val = line.strip().partition("=")
            if sep and not key.startswith("#"):
                env[key.strip()] = val.strip()
    return env


def upsert_env(values):
    """Sets keys of .env that start.sh does not set (the formation mission)."""
    path = os.path.join(ROOT, ".env")
    with open(path) as f:
        lines = f.read().splitlines()
    for key, val in values.items():
        for i, line in enumerate(lines):
            if line.strip().startswith(f"{key}="):
                lines[i] = f"{key}={val}"
                break
        else:
            lines.append(f"{key}={val}")
    with open(path, "w") as f:
        f.write("\n".join(lines) + "\n")


def own(path):
    """The stack's containers write as root: hand path back to this user."""
    subprocess.run(["docker", "run", "--rm", "-v", f"{path}:/d", "alpine", "chown", "-R",
                    f"{os.getuid()}:{os.getgid()}", "/d"], capture_output=True)


def stop_stack():
    subprocess.run([os.path.join(ROOT, "stop.sh")], cwd=ROOT, capture_output=True, text=True)


def container_running(name):
    out = subprocess.run(["docker", "ps", "--format", "{{.Names}}"], capture_output=True, text=True)
    return name in out.stdout.split()


def wait_goal_published(timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        if container_running("orchestrator"):
            out = subprocess.run(["docker", "logs", "orchestrator"], capture_output=True, text=True)
            if "Goal published" in out.stdout + out.stderr:
                return True
        time.sleep(2)
    return False


def measurements():
    r = requests.get(f"{VICTIM_SIDECAR}/status", timeout=5)
    r.raise_for_status()
    return r.json()["counters"]["measurements"]


def wait_measurement_end(ssp_ms):
    """Returns within 0.1 s of robot3 finishing its next measurement."""
    deadline = time.time() + ssp_ms / 1000 + 30
    count = measurements()
    while measurements() == count:
        if time.time() > deadline:
            raise RuntimeError("robot3 does not measure: is its sidecar attesting?")
        time.sleep(0.1)


def estimate_retries():
    """Gas estimates the four sidecars tried again after a Besu "Internal
    error" since the stack started, i.e. in this run (each adds ~100 ms)."""
    total = 0
    for port in range(8001, 8005):
        try:
            r = requests.get(f"http://localhost:{port}/status", timeout=5)
            total += r.json()["counters"].get("gas_estimate_retries", 0)
        except (requests.RequestException, ValueError, KeyError):
            return None
    return total


def tamper_victim():
    out = subprocess.run([os.path.join(ROOT, "tools", "tamper.sh"), VICTIM, "--target", "formation_agent"],
                         capture_output=True, text=True)
    m = TAMPERED_RE.search(out.stdout)
    if out.returncode != 0 or not m:
        raise RuntimeError(f"tamper.sh failed:\n{out.stdout}{out.stderr}")
    return int(m.group(1))


def compute_metrics():
    """tools/formation_metrics.py on the staged bag, in the ROS image."""
    rel = os.path.relpath(STAGING, ROOT)
    out = subprocess.run(
        ["docker", "run", "--rm", "--user", "root", "-v", f"{ROOT}:/w", "-w", "/w", "robot-simulation", "bash", "-lc",
         f"source /opt/ros/humble/setup.bash && python3 tools/formation_metrics.py --bag {rel}/bags/{TAG} "
         f"--events {rel}/events-{TAG}.json --out-dir {rel}"],
        capture_output=True, text=True)
    return out.returncode == 0, (out.stdout + out.stderr)[-2000:]


def scenario_dir(name):
    return os.path.join(OUT_ROOT, name)


def check_scenario(base, setup):
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


def one_run(name, ssp, mitigation, attack, args, setup):
    log(f"▶ {name}: SSP {ssp} ms, mitigation {'on' if mitigation else 'off'}, attack {'yes' if attack else 'no'}")
    if os.path.isdir(STAGING):
        own(STAGING)
        shutil.rmtree(STAGING)
    os.makedirs(os.path.join(STAGING, "bags"))
    upsert_env({"GOAL_X": args.goal_x, "GOAL_Y": 0.0, "RUN_DURATION_S": args.window, "K_FORM": args.k_form,
                "K_ATT": args.k_att, "POLL_INTERVAL_S": args.poll_interval, "FORMATION_DATA_SUBDIR": STAGING_SUBDIR})
    cmd = [os.path.join(ROOT, "start.sh"), "--robots", "4", "--formation", "--auto", "--iterq", "1",
           "--mitigation", "on" if mitigation else "off", "--run-tag", TAG, "--ssp", str(ssp)]
    if args.gpu:
        cmd.append("--gpu")
    t_deploy = time.time()
    started = subprocess.run(cmd, cwd=ROOT, capture_output=True, text=True,
                             env={**os.environ, "FORMATION_DATA_SUBDIR": STAGING_SUBDIR})
    with open(os.path.join(STAGING, "start.log"), "w") as f:
        f.write(started.stdout + started.stderr)
    problems = []
    t_tamper_ms = None
    try:
        if started.returncode != 0:
            raise RuntimeError(f"start.sh failed (see {os.path.relpath(STAGING, ROOT)}/start.log)")
        log(f"   stack up in {time.time() - t_deploy:.0f} s; waiting for the goal")
        if not wait_goal_published(args.goal_timeout):
            raise RuntimeError("the orchestrator never published the goal")
        if not container_running("bag-recorder"):
            raise RuntimeError("the bag recorder is not running: no data would be captured")
        t_goal = time.time()
        log("   mission started")
        if attack:
            time.sleep(args.attack_delay)
            wait_measurement_end(ssp)
            t_tamper_ms = tamper_victim()
            alive = subprocess.run(["docker", "exec", VICTIM, "pgrep", "formation_agent"], capture_output=True)
            if alive.returncode != 0:
                problems.append("the tampered formation_agent died: no failure can be measured")
            log(f"   tampered with {VICTIM} at {t_tamper_ms / 1000 - t_goal:.1f} s into the mission")
        events_path = os.path.join(STAGING, f"events-{TAG}.json")
        deadline = t_goal + args.window + 120
        while not os.path.exists(events_path):
            if time.time() > deadline:
                raise RuntimeError("the orchestrator never wrote its events: window not closed")
            time.sleep(2)
        time.sleep(2)  # the file is written at once; let it close
    except (KeyboardInterrupt, Exception):
        log("   tearing the stack down; the run is not written")
        stop_stack()
        raise
    retries = estimate_retries()
    log("   window closed; tearing down (writes the bag)")
    stop_stack()
    own(STAGING)

    if t_tamper_ms is not None:
        with open(os.path.join(STAGING, f"inject-{TAG}.json"), "w") as f:
            json.dump({"tag": TAG, "victim": VICTIM, "attack_delay_s": args.attack_delay, "anchored": True,
                       "t_inject_wall": datetime.fromtimestamp(t_tamper_ms / 1000, timezone.utc)
                       .strftime("%Y-%m-%dT%H:%M:%S.%f")[:-3] + "Z"}, f)
    bag = os.path.join(STAGING, "bags", TAG)
    if not os.path.isfile(os.path.join(bag, "metadata.yaml")):
        problems.append("no complete bag")
    ok, output = compute_metrics()
    own(STAGING)
    if not ok:
        problems.append(f"formation_metrics.py failed: {output[-300:]}")

    with open(os.path.join(STAGING, f"events-{TAG}.json")) as f:
        events = json.load(f)
    metrics = {}
    metrics_path = os.path.join(STAGING, f"metrics-{TAG}.json")
    if os.path.isfile(metrics_path):
        with open(metrics_path) as f:
            metrics = json.load(f)
    failed = [r for _, r in events.get("failure_events", [])]
    if attack and VICTIM not in failed:
        problems.append(f"no failure verdict for {VICTIM} within the window")
    if not attack and failed:
        problems.append(f"failure verdicts without attack: {failed}")
    if failed and any(r != VICTIM for r in failed):
        problems.append(f"failure verdicts for healthy robots: {failed}")
    if mitigation and attack and metrics and metrics.get("t_detect_s") is None:
        problems.append("robot3 was not excluded (no /formation/excluded in the bag)")

    meta = {
        **setup, "scenario": name, "ssp_ms": ssp, "mitigation": mitigation, "attack": attack,
        "t_deploy": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_deploy)),
        "t_tamper_ms": t_tamper_ms, "failure_events": events.get("failure_events", []),
        "t_inject_s": metrics.get("t_inject_s"), "t_detect_s": metrics.get("t_detect_s"),
        "t_attack_to_detect_s": metrics.get("t_attack_to_detect_s"),
        "progress_pct": metrics.get("progress_pct"), "cumulative_damage": metrics.get("cumulative_damage"),
        "gas_estimate_retries": retries,
        "problems": problems,
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }

    base = scenario_dir(name)
    os.makedirs(base, exist_ok=True)
    taken = [int(d[3:]) for d in os.listdir(base) if d.startswith("run") and d[3:].isdigit()]
    path = os.path.join(base, f"run{max(taken, default=0) + 1:02d}")
    tmp = path + ".tmp"
    os.makedirs(tmp)
    moves = {f"bags/{TAG}": "bag", f"events-{TAG}.json": "events.json", f"inject-{TAG}.json": "inject.json",
             f"formation-{TAG}.csv": "formation.csv", f"metrics-{TAG}.json": "metrics.json", "start.log": "start.log"}
    for src, dst in moves.items():
        if os.path.exists(os.path.join(STAGING, src)):
            os.rename(os.path.join(STAGING, src), os.path.join(tmp, dst))
    with open(os.path.join(tmp, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    os.rename(tmp, path)
    log(f"   progress {meta['progress_pct']}%, damage {meta['cumulative_damage']}, "
        f"attack→detect {meta['t_attack_to_detect_s']} s, {len(problems)} problem(s) → {os.path.relpath(path, ROOT)}")
    for p in problems:
        log(f"   ⚠️  {p}")
    log(f"   cooldown {args.cooldown} s")
    time.sleep(args.cooldown)


def main():
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--reps", type=int, default=3, help="repetitions of every scenario (default: 3)")
    p.add_argument("--scenarios", default="baseline,unmitigated,mitigated",
                   help="scenarios to run (default: baseline,unmitigated,mitigated)")
    p.add_argument("--mitigated-ssp", default="10000,40000", help="SSPs of the mitigated runs (default: 10000,40000)")
    p.add_argument("--unmitigated-ssp", type=int, default=10000, help="SSP of the unmitigated runs (default: 10000)")
    p.add_argument("--baseline-ssp", type=int, default=10000, help="SSP of the baseline runs (default: 10000)")
    p.add_argument("--goal-x", type=float, default=60.0, help="goal distance in m (default: 60)")
    p.add_argument("--window", type=int, default=400, help="mission window in s after the goal (default: 400)")
    p.add_argument("--attack-delay", type=int, default=60, help="s of travel before the attack (default: 60)")
    p.add_argument("--k-form", type=float, default=1.5, help="formation cohesion gain (default: 1.5)")
    p.add_argument("--k-att", type=float, default=0.8, help="goal attraction gain (default: 0.8)")
    p.add_argument("--poll-interval", type=float, default=0.2, help="orchestrator chain check, s (default: 0.2)")
    p.add_argument("--gpu", action="store_true", help="Gazebo on the NVIDIA GPU (needs the container toolkit)")
    p.add_argument("--goal-timeout", type=int, default=360, help="s to wait for the mission start (default: 360)")
    p.add_argument("--cooldown", type=int, default=60, help="s of rest after each run (default: 60)")
    args = p.parse_args()

    runs = []
    for scn in args.scenarios.split(","):
        if scn == "baseline":
            runs.append(("baseline", args.baseline_ssp, True, False))
        elif scn == "unmitigated":
            runs.append((f"unmitigated-SSP{args.unmitigated_ssp}", args.unmitigated_ssp, False, True))
        elif scn == "mitigated":
            runs += [(f"mitigated-SSP{s}", int(s), True, True) for s in args.mitigated_ssp.split(",")]
        else:
            sys.exit(f"unknown scenario: {scn}")
    if args.attack_delay + 120 > args.window:
        sys.exit("--window must leave room after the attack for detection and recovery (attack delay + 120 s)")

    env = read_env()
    setup = {
        "contract": "AttestationManagerV3", "robots": 4, "victim": f"{VICTIM} formation_agent (constant-turn fault)",
        "goal_x": args.goal_x, "window_s": args.window, "attack_delay_s": args.attack_delay, "k_form": args.k_form,
        "k_att": args.k_att, "formation_scale": float(env.get("FORMATION_SCALE") or 2.0),
        "poll_interval_s": args.poll_interval, "gpu": args.gpu, "sidecar_cpu_limit": env.get("CPU_LIMIT", "0.4"),
        "iterq": 1, "jury_size": int(env.get("JURY_SIZE") or 2), "jury_threshold": int(env.get("JURY_THRESHOLD") or 2),
    }
    for name, *_ in runs:
        check_scenario(scenario_dir(name), setup)

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")

    log(f"tamper impact: {len(runs)} scenario(s) x {args.reps} rep(s): {', '.join(r[0] for r in runs)}")
    log(f"   goal {args.goal_x} m, window {args.window} s, attack after {args.attack_delay} s, "
        f"k_form {args.k_form}, k_att {args.k_att}, gpu {args.gpu}")
    log("   the stack is redeployed for every run: the current one is torn down now")
    stop_stack()
    failed = []
    try:
        for rep in range(1, args.reps + 1):
            order = random.sample(runs, len(runs))
            log(f"── repetition {rep}/{args.reps}: {', '.join(r[0] for r in order)}")
            for name, ssp, mitigation, attack in order:
                # A failed run is skipped, not written: the campaign goes on.
                try:
                    one_run(name, ssp, mitigation, attack, args, setup)
                except RuntimeError as e:
                    failed.append(f"{name} (repetition {rep}): {e}")
                    log(f"   ❌ run failed, skipped: {e}")
    except KeyboardInterrupt:
        log("interrupted")
        sys.exit(130)
    if os.path.isdir(STAGING):
        own(STAGING)
        shutil.rmtree(STAGING)
    log(f"done; the stack is down. {len(failed)} run(s) failed" + "".join(f"\n   - {f}" for f in failed))
    sys.exit(1 if failed else 0)


if __name__ == "__main__":
    main()
