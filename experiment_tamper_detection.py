#!/usr/bin/env python3
"""Tamper detection experiment (contract V3): how long after the attested
robot process is tampered with does the protocol detect it?

Setup: Gazebo robots, IterQ 1, e.g.
  ./start.sh --robots 4 --iterq 1

robot1's robot_state_publisher, the attested process, is tampered with in
memory (tools/tamper.sh: one byte of its code overwritten) right after a
clean measurement: the worst case, since the tampering is first measured one
SSP later. Detection is the jury's failure verdict on that measurement's
round. The time until a random-time attack is detected is shorter by the
wait before the next measurement.

Each trial: wait until robot1 finishes a measurement, tamper at once, wait
for the verdict on the next measurement, restore the original bytes. The
measurement after the restore is clean and its round must succeed (the
recovery); the next trial tampers right after it. All the robots attest at
the condition's SSP; robot1 is the target.

Times come from the robot host only (tamper.sh runs in robot1's sidecar
container; the prover's rows are timed there), so no clock synchronization
is needed:
  latency = wait for the next measurement + its round (evidence, presend,
            send, inclusion, judgement)

Conditions: --ssp values, in random order. Each starts after a chain reset
and one round per robot (every robot eligible as a juror), and ends with the
robots stopped and a rest.

Written to experiments/data/tamper-detection/SSP<ms>/runNN/:
  meta.json   setup, condition, block range, trials, problems
  trials.csv  one row per trial: tamper and restore times, the detecting
              round (its id, measurement start, steps, verdict, judge,
              votes), the latency, whether the recovery round succeeded
Runs are appended (runNN numbering continues) only when the folder's runs
have the same setup. One experiment runs at a time; Ctrl-C restores robot1's
bytes, stops the robots and the condition in progress is not written.

Example:
  python3 experiment_tamper_detection.py --trials 10
"""

import argparse
import csv
import fcntl
import json
import os
import random
import re
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import requests

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "tamper-detection")
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
TAMPER = os.path.join(ROOT, "tools", "tamper.sh")
RPC_URL = "http://localhost:21001"  # validator1
SECAAS = ("secaas", "localhost", 8000)
ROBOTS_BASE_PORT = 8001
TARGET_ROBOT = "robot1"
ADMIN_KEYS = [os.path.join(ROOT, "blockchain/quorum-test-network/config/nodes", v, "accountPrivateKey")
              for v in ("validator2", "validator3")]
TAMPERED_RE = re.compile(r"tampered .* at (\d+) ms")
RESTORED_RE = re.compile(r"restored the original bytes at (\d+) ms")

TRIAL_COLUMNS = ["trial", "t_tamper_ms", "t_restore_ms", "round_id", "t_measure_ms", "wait_ms", "evidence_ms",
                 "presend_ms", "send_ms", "inclusion_ms", "judgement_ms", "round_ms", "latency_ms", "verdict",
                 "closed_by", "votes", "detected", "recovered"]
# meta.json keys that must be equal for all the runs of a condition folder.
SETUP_KEYS = ["contract", "robots", "target", "gazebo_world", "sidecar_cpu_limit", "jury_size", "jury_threshold",
              "block_period_s", "iterq"]

HTTP = requests.Session()


# ── setup ─────────────────────────────────────────────────────────────────────

def read_env():
    env = {}
    with open(os.path.join(ROOT, ".env")) as f:
        for line in f:
            key, sep, val = line.strip().partition("=")
            if sep and not key.startswith("#"):
                env[key.strip()] = val.strip()
    return env


def head():
    r = HTTP.post(RPC_URL, json={"jsonrpc": "2.0", "id": 1, "method": "eth_blockNumber", "params": []}, timeout=10)
    r.raise_for_status()
    return int(r.json()["result"], 16)


def wait_blocks(n):
    target = head() + n
    while head() < target:
        time.sleep(0.2)


def chain(*args):
    """Runs tools/chain.py in the secaas container."""
    with open(os.path.join(ROOT, "tools", "chain.py")) as script:
        out = subprocess.run(["docker", "exec", "-i", "secaas", "python3", "-", *map(str, args)],
                             stdin=script, capture_output=True, text=True, check=True)
    return out.stdout


def reset_chain(contract):
    out = subprocess.run([os.path.join(ROOT, "reset_chain.sh"), contract, *ADMIN_KEYS],
                         capture_output=True, text=True)
    if out.returncode != 0:
        sys.exit(f"chain reset failed:\n{out.stdout}{out.stderr}")
    wait_blocks(2)


def tamper(*flags):
    """Runs tools/tamper.sh on robot1's attested process; returns its output."""
    out = subprocess.run([TAMPER, TARGET_ROBOT, "--target", "state_publisher", *flags],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"tamper.sh {' '.join(flags)} failed:\n{out.stdout}{out.stderr}")
    return out.stdout


# ── agents ────────────────────────────────────────────────────────────────────

def call(agent, method, path, **kw):
    _, host, port = agent
    r = HTTP.request(method, f"http://{host}:{port}{path}", timeout=kw.pop("timeout", 10), **kw)
    r.raise_for_status()
    return r.json()


def each(items, fn):
    with ThreadPoolExecutor(max_workers=max(1, len(items))) as pool:
        return list(pool.map(fn, items))


def start_together(agents):
    barrier = threading.Barrier(len(agents))
    def start(agent):
        barrier.wait()
        call(agent, "POST", "/start")
    each(agents, start)


def estimate_retries(agents):
    """Gas estimates the agents tried again after a Besu "Internal error"
    (each adds ~100 ms to a step): recorded per run, to flag runs if needed."""
    return sum(each(agents, lambda a: call(a, "GET", "/status")["counters"].get("gas_estimate_retries", 0)))


def stop_robots(robots):
    each(robots, lambda a: call(a, "POST", "/stop", timeout=60))


def configure(robots, rounds, ssp, offsets):
    each(list(zip(robots, offsets)), lambda ro: call(ro[0], "POST", "/config", json={
        "rounds": rounds, "start_offset_ms": ro[1], "attestation_interval_ms": ssp,
        "attestation_interval_mode": "fixed", "iterq_threshold": 1}))


def warm_up(env, robots):
    """One round per robot after a chain reset: every robot then eligible."""
    print("   warm-up: chain reset, one round per robot, not recorded")
    reset_chain(env["CONTRACT"])
    configure(robots, 1, 10000, [0] * len(robots))
    start_together(robots)
    deadline = time.time() + 120
    while time.time() < deadline:
        if all(s == "finished" for s in each(robots, lambda a: call(a, "GET", "/status")["status"])):
            break
        time.sleep(2)
    stop_robots(robots)
    wait_blocks(2)
    if int(chain("eligible")) < len(robots):
        sys.exit("warm-up: not every robot is eligible. If the sidecar image was rebuilt, its references "
                 "changed: run ./start.sh again.")


def counter(target):
    return call(target, "GET", "/status")["counters"]["measurements"]


def wait_clean_measurement(target, ssp, not_before_ms, last_end):
    """Returns (host time) right after robot1 finishes a measurement that
    started after not_before_ms, i.e. a clean one, taken after the last
    restore. Polls its measurement counter every 20 ms only near the expected
    end (last_end + SSP), to load the sidecar being measured as little as
    possible."""
    count = counter(target)
    deadline = time.time() + 2 * ssp / 1000 + 30
    while True:
        wake = (last_end or 0) + ssp / 1000 - 0.5
        if time.time() < wake:
            time.sleep(min(0.5, wake - time.time()))
            continue
        now_count = counter(target)
        if now_count != count:
            now = time.time()
            if now * 1000 - 300 > not_before_ms:  # a measurement takes ~100 ms
                return now
            count, last_end = now_count, now      # it began before the restore: the next one
        if time.time() > deadline:
            raise RuntimeError(f"SSP {ssp}: robot1 stopped measuring")
        time.sleep(0.02)


def measure_start_ms(row):
    """Wall-clock ms when the round's measurement started (the sidecar's clock)."""
    return row["t_sent_ms"] - row.get("presend_ms", 0) - row["evidence_ms"]


def closed_ms(row):
    return row["t_sent_ms"] + row.get("send_ms", 0) + row.get("inclusion_ms", 0) + row.get("judgement_ms", 0)


# ── one condition ─────────────────────────────────────────────────────────────

def condition_dir(ssp):
    return os.path.join(OUT_ROOT, f"SSP{ssp}")


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


def write_run(base, rows, meta):
    """Writes the run to the next runNN; a run is only named once complete."""
    os.makedirs(base, exist_ok=True)
    taken = [int(d[3:]) for d in os.listdir(base) if d.startswith("run") and d[3:].isdigit()]
    name = f"run{max(taken, default=0) + 1:02d}"
    tmp = os.path.join(base, f".{name}.tmp")
    os.makedirs(tmp)
    with open(os.path.join(tmp, "trials.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=TRIAL_COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(tmp, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    os.rename(tmp, os.path.join(base, name))
    return os.path.join(base, name)


def prover_rows(target):
    return [r for r in call(target, "GET", "/results")["prover"] if r.get("outcome")]


def run_condition(ssp, args, setup, env, robots):
    target = robots[0]
    warm_up(env, robots)
    each(robots, lambda a: call(a, "DELETE", "/results"))
    retries = estimate_retries(robots)
    offsets = [0] + [round(random.uniform(0, ssp)) for _ in robots[1:]]
    configure(robots, 0, ssp, offsets)
    first = head() + 1
    t_start = time.time()
    start_together(robots)

    # Healthy first: one successful round of robot1.
    deadline = time.time() + ssp / 1000 + 30
    while not any(r["outcome"] == "success" for r in prover_rows(target)):
        if time.time() > deadline:
            raise RuntimeError(f"SSP {ssp}: robot1 has no successful round: check its sidecar's logs")
        time.sleep(0.5)

    trials = []
    not_before_ms, last_end = 0, None
    for i in range(1, args.trials + 1):
        # Right after robot1 finishes a clean measurement, tamper.
        last_end = wait_clean_measurement(target, ssp, not_before_ms, last_end)
        t_tamper = int(TAMPERED_RE.search(tamper()).group(1))

        # The verdict on the next measurement, then the restore.
        deadline = time.time() + ssp / 1000 + 60
        while True:
            after = [r for r in prover_rows(target) if measure_start_ms(r) > t_tamper]
            if after or time.time() > deadline:
                break
            time.sleep(0.3)
        t_restore = int(RESTORED_RE.search(tamper("--restore")).group(1))
        not_before_ms = t_restore
        if after:  # the tampered measurement ended then: the next ends one SSP later
            last_end = (measure_start_ms(after[0]) + after[0]["evidence_ms"]) / 1000
        outcome = after[0]["outcome"] if after else "none"
        print(f"   [{i}/{args.trials}] {outcome}" + (f" after {(closed_ms(after[0]) - t_tamper) / 1000:.3f} s"
                                                     if after else ""))
        trials.append({"trial": i, "t_tamper_ms": t_tamper, "t_restore_ms": t_restore})

    # The recovery of the last trial: robot1's next round succeeds.
    deadline = time.time() + ssp / 1000 + 30
    while not any(measure_start_ms(r) > trials[-1]["t_restore_ms"] for r in prover_rows(target)):
        if time.time() > deadline:
            break
        time.sleep(0.5)
    rows = sorted(prover_rows(target), key=measure_start_ms)
    stop_robots(robots)
    wait_blocks(3)
    last = head()
    data = json.loads(chain("extract", first, last))
    rounds = {r["round_id"]: r for r in data["rounds"]}
    time.sleep(args.cooldown)

    problems = []
    for t in trials:
        detecting = next((r for r in rows if measure_start_ms(r) > t["t_tamper_ms"]), None)
        recovery = next((r for r in rows if measure_start_ms(r) > t["t_restore_ms"]), None)
        t["recovered"] = int(recovery is not None and recovery["outcome"] == "success")
        if detecting is None:
            t["detected"] = 0
            problems.append(f"trial {t['trial']}: no round measured after the tampering")
            continue
        chain_round = rounds.get(detecting["round_id"], {})
        ms = lambda key: round(detecting[key], 3) if key in detecting else ""  # noqa: E731
        t.update({
            "round_id": detecting["round_id"], "t_measure_ms": round(measure_start_ms(detecting)),
            "wait_ms": round(measure_start_ms(detecting) - t["t_tamper_ms"]),
            "evidence_ms": ms("evidence_ms"), "presend_ms": ms("presend_ms"), "send_ms": ms("send_ms"),
            "inclusion_ms": ms("inclusion_ms"), "judgement_ms": ms("judgement_ms"), "round_ms": ms("total_ms"),
            "latency_ms": round(closed_ms(detecting) - t["t_tamper_ms"]), "verdict": detecting["outcome"],
            "closed_by": chain_round.get("closed_by", ""), "votes": chain_round.get("votes", ""),
            "detected": int(detecting["outcome"] == "failure"),
        })
        if not t["detected"]:
            problems.append(f"trial {t['trial']}: verdict {detecting['outcome']} on the tampered measurement")
        if t["closed_by"] != "jury":
            problems.append(f"trial {t['trial']}: closed by {t['closed_by'] or 'nobody'}")
        if not t["recovered"]:
            problems.append(f"trial {t['trial']}: robot1's round after the restore did not succeed")
        if detecting.get("presend_ms", 0) > 1000:
            problems.append(f"trial {t['trial']}: the round waited {detecting['presend_ms']:.0f} ms to be sent")
    detected = sum(t.get("detected", 0) for t in trials)
    latencies = sorted(t["latency_ms"] for t in trials if t.get("detected"))
    print(f"   SSP {ssp}: {detected}/{len(trials)} detected"
          + (f", latency {latencies[0] / 1000:.2f}..{latencies[-1] / 1000:.2f} s" if latencies else "")
          + f", {len(problems)} problem(s)")
    for p in problems:
        print(f"   ⚠️  {p}")

    meta = {
        **setup, "ssp_ms": ssp, "trials": len(trials), "detected": detected,
        "start_offsets_ms": dict(zip((a[0] for a in robots), offsets)), "first_block": first, "last_block": last,
        "gas_estimate_retries": estimate_retries(robots) - retries,
        "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_start)), "problems": problems,
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }
    path = write_run(condition_dir(ssp), trials, meta)
    print(f"   → {os.path.relpath(path, ROOT)}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ssp", default="5000,10000,20000,40000", help="SSPs in ms (default: 5000,10000,20000,40000)")
    p.add_argument("--trials", type=int, default=10, help="trials per SSP (default: 10)")
    p.add_argument("--cooldown", type=int, default=10, help="seconds of rest after each condition (default: 10)")
    args = p.parse_args()
    ssps = [int(s) for s in args.ssp.split(",")]

    if env.get("CONTRACT") != "AttestationManagerV3":
        sys.exit("experiment_tamper_detection.py measures contract V3: deploy it with ./start.sh --contract v3")
    if env.get("DUMMY") == "TRUE" or env.get("DEPLOY_MODE") == "remote" or int(env.get("N_ROBOTS", 0)) < 4:
        sys.exit("experiment_tamper_detection.py needs local Gazebo robots: ./start.sh --robots 4 --iterq 1")
    if env.get("WAIT_FOR_VERIFICATION_RESULT", "TRUE") != "TRUE":
        sys.exit("the provers must wait for the verdicts: ./start.sh without --no-wait-result")
    if min(ssps) < 5000:
        sys.exit("SSP below 5 s: with IterQ 1 a prover has one round open at a time (~3 s), so measurements "
                 "would queue behind it and the latency would be the queue's")
    with open(os.path.join(ROOT, "blockchain/quorum-test-network/config/besu/genesis.json")) as f:
        block_period = json.load(f)["config"]["qbft"]["blockperiodseconds"]
    # A round left open by the stop expires to the SECaaS: the rest outlasts it.
    args.cooldown = max(args.cooldown, (int(env["JURY_TIMEOUT_BLOCKS"]) + 5) * block_period)

    n = int(env["N_ROBOTS"])
    robots = [(f"robot{i}", "localhost", ROBOTS_BASE_PORT + i - 1) for i in range(1, n + 1)]
    setup = {
        "contract": env["CONTRACT"], "robots": n, "target": f"{TARGET_ROBOT} robot_state_publisher",
        "tampering": "1 byte of 0x90 (NOP) at .text + 0x2000 (tools/tamper.sh)",
        "gazebo_world": env.get("GAZEBO_WORLD"), "sidecar_cpu_limit": env.get("CPU_LIMIT"),
        "jury_size": int(env["JURY_SIZE"]), "jury_threshold": int(env["JURY_THRESHOLD"]),
        "block_period_s": block_period, "iterq": 1, "cooldown_s": args.cooldown,
    }
    for ssp in ssps:
        check_condition(condition_dir(ssp), setup)

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")

    order = random.sample(ssps, len(ssps))
    print(f"setup: {n} Gazebo robots, target {setup['target']}, SSP {order} ms, {args.trials} trials each")
    try:
        each(robots + [SECAAS], lambda a: call(a, "GET", "/"))  # all reachable
        stop_robots(robots)
        call(SECAAS, "POST", "/stop", timeout=120)  # a fresh SECaaS loop, kept running
        call(SECAAS, "POST", "/start")
        for ssp in order:
            print(f"── SSP {ssp} ms")
            run_condition(ssp, args, setup, env, robots)
    except (KeyboardInterrupt, Exception) as e:
        # robot1 is never left tampered with, whatever stops the experiment.
        print(f"\n{'interrupted' if isinstance(e, KeyboardInterrupt) else repr(e)}\n"
              "the condition in progress is not written; restoring robot1 and stopping the robots")
        try:
            tamper("--restore")
        except RuntimeError:
            pass  # nothing saved: already restored
        stop_robots(robots)
        sys.exit(130 if isinstance(e, KeyboardInterrupt) else 1)


if __name__ == "__main__":
    main()
