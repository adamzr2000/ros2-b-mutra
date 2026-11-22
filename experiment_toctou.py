#!/usr/bin/env python3
"""TOCTOU experiment (contract V3): how often is a tamper-run-restore attack
of duration tau caught, by tau / SSP and by measurement schedule?

The attacker is informed: it waits for the end of a measurement, tampers
with the attested process at once and restores the original bytes tau later
(tools/tamper.sh --restore-after), hoping that no measurement happens in
between. It is caught when a measurement started between the tampering and
the restore gets a failure verdict.
  fixed  the next measurement is exactly one SSP later: caught only when
         tau >= SSP
  exp    random delays of mean SSP (exponential): caught with probability
         about 1 - exp(-tau / SSP), however short the attack

Setup: Gazebo robots, IterQ 1, one robot per (SSP, mode) point, e.g.
  ./start.sh --robots 4 --iterq 1
The points of --ssp x --modes go to robot1, robot2, ... and run at the same
time, each robot its own trials: every ratio of --ratios --trials times, in
a shuffled order. They start after a chain reset and one round per robot
(every robot eligible as a juror).

Times come from each robot's host (tamper.sh runs in the robot's sidecar
container; the prover's rows are timed there): no clock synchronization. The
measurements of an attack window are read from the prover's rows
(t_measured_ms), with their verdicts: failure, success, or stale (waited
past the freshness window behind an open round: never judged).

Each point is written to experiments/data/toctou/<mode>-SSP<ms>/runNN/:
  meta.json   setup, point, trials, problems
  trials.csv  one row per trial: ratio, tau, the end of the measurement the
              attacker waited for, tampering and restore times, the
              measurements in the window and their verdicts, detected
trials.csv is filled as the trials go (in .runNN.tmp/ until the point ends).
One experiment runs at a time; Ctrl-C or an error stops every point, the
robots are restored by tamper.sh itself, and the robots stop.

Example (~2 h):
  python3 experiment_toctou.py --trials 20
"""

import argparse
import csv
import fcntl
import itertools
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
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "toctou")
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
TAMPER = os.path.join(ROOT, "tools", "tamper.sh")
RPC_URL = "http://localhost:21001"  # validator1
SECAAS = ("secaas", "localhost", 8000)
ROBOTS_BASE_PORT = 8001
ADMIN_KEYS = [os.path.join(ROOT, "blockchain/quorum-test-network/config/nodes", v, "accountPrivateKey")
              for v in ("validator2", "validator3")]
TAMPERED_RE = re.compile(r"tampered .* at (\d+) ms")
RESTORED_RE = re.compile(r"restored the original bytes at (\d+) ms")

TRIAL_COLUMNS = ["trial", "ratio", "tau_ms", "t_anchor_ms", "t_tamper_ms", "t_restore_ms", "measurements",
                 "failures", "successes", "stale", "other", "verdicts", "detected", "complete"]
JUDGED = {"success", "failure"}

HTTP = requests.Session()


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


def head():
    r = HTTP.post(RPC_URL, json={"jsonrpc": "2.0", "id": 1, "method": "eth_blockNumber", "params": []}, timeout=10)
    r.raise_for_status()
    return int(r.json()["result"], 16)


def wait_blocks(n):
    target = head() + n
    while head() < target:
        time.sleep(0.2)


def chain(*args):
    with open(os.path.join(ROOT, "tools", "chain.py")) as script:
        out = subprocess.run(["docker", "exec", "-i", "secaas", "python3", "-", *map(str, args)],
                             stdin=script, capture_output=True, text=True, check=True)
    return out.stdout


def reset_chain(contract):
    out = subprocess.run([os.path.join(ROOT, "reset_chain.sh"), contract, *ADMIN_KEYS],
                         capture_output=True, text=True)
    if out.returncode != 0:
        raise RuntimeError(f"chain reset failed:\n{out.stdout}{out.stderr}")
    wait_blocks(2)


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


def warm_up(env, robots):
    """One round per robot after a chain reset: every robot then eligible."""
    log("warm-up: chain reset, one round per robot, not recorded")
    reset_chain(env["CONTRACT"])
    each(robots, lambda a: call(a, "POST", "/config", json={
        "rounds": 1, "start_offset_ms": 0, "attestation_interval_ms": 10000,
        "attestation_interval_mode": "fixed", "iterq_threshold": 1}))
    start_together(robots)
    deadline = time.time() + 120
    while time.time() < deadline:
        if all(s == "finished" for s in each(robots, lambda a: call(a, "GET", "/status")["status"])):
            break
        time.sleep(2)
    stop_robots(robots)
    wait_blocks(2)
    if int(chain("eligible")) < len(robots):
        raise RuntimeError("warm-up: not every robot is eligible. If the sidecar image was rebuilt, its "
                           "references changed: run ./start.sh again.")


# ── one point ─────────────────────────────────────────────────────────────────

class Point:
    """One robot attesting at one (SSP, mode), with its trials."""

    def __init__(self, robot, ssp, mode, plan, setup, stop):
        self.robot, self.ssp, self.mode, self.plan, self.setup, self.stop = robot, ssp, mode, plan, setup, stop
        self.name = f"{mode}-SSP{ssp}"
        self.trials = []

    def counter(self):
        return call(self.robot, "GET", "/status")["counters"]["measurements"]

    def rows(self):
        return [r for r in call(self.robot, "GET", "/results")["prover"] if r.get("outcome")]

    def wait_measurement_end(self, not_before_ms, last_end):
        """Returns (host time) right after the robot finishes a measurement
        that started after not_before_ms. With a fixed SSP the next end is
        known: the counter is polled every 20 ms only near it."""
        count = self.counter()
        deadline = time.time() + 10 * self.ssp / 1000 + 60
        while not self.stop.is_set():
            if self.mode == "fixed" and last_end:
                wake = last_end + self.ssp / 1000 - 0.5
                if time.time() < wake:
                    time.sleep(min(0.5, wake - time.time()))
                    continue
            now_count = self.counter()
            if now_count != count:
                now = time.time()
                if now * 1000 - 300 > not_before_ms:  # a measurement takes ~100 ms
                    return now
                count, last_end = now_count, now      # it began before the restore: the next one
            if time.time() > deadline:
                raise RuntimeError(f"{self.robot[0]}: no measurement for {10 * self.ssp / 1000 + 60:.0f} s")
            time.sleep(0.02)
        return None

    def window_rows(self, t_tamper, t_restore):
        """The rows of the measurements started in the attack window, once
        every one of them has its row: the rounds are handled in order, so
        the row of a measurement started after the restore closes the set."""
        deadline = time.time() + 5 * self.ssp / 1000 + 120
        while not self.stop.is_set():
            rows = self.rows()
            if any(r["t_measured_ms"] > t_restore for r in rows):
                return [r for r in rows if t_tamper < r["t_measured_ms"] <= t_restore], True
            if time.time() > deadline:
                return [r for r in rows if t_tamper < r["t_measured_ms"] <= t_restore], False
            time.sleep(0.5)
        return [], False

    def run(self, out):
        not_before_ms, last_end = 0, None
        for n, ratio in enumerate(self.plan, 1):
            if self.stop.is_set():
                return
            tau_ms = round(ratio * self.ssp)
            anchor = self.wait_measurement_end(not_before_ms, last_end)
            if anchor is None:
                return
            res = subprocess.run([TAMPER, self.robot[0], "--target", "state_publisher", "--restore-after",
                                  str(tau_ms)], capture_output=True, text=True, timeout=tau_ms / 1000 + 60)
            t, r = TAMPERED_RE.search(res.stdout), RESTORED_RE.search(res.stdout)
            if res.returncode != 0 or not (t and r):
                raise RuntimeError(f"{self.robot[0]}: tamper.sh failed:\n{res.stdout}{res.stderr}")
            t_tamper, t_restore = int(t.group(1)), int(r.group(1))
            rows, complete = self.window_rows(t_tamper, t_restore)
            verdicts = [row["outcome"] for row in rows]
            trial = {
                "trial": n, "ratio": ratio, "tau_ms": tau_ms, "t_anchor_ms": round(anchor * 1000),
                "t_tamper_ms": t_tamper, "t_restore_ms": t_restore, "measurements": len(rows),
                "failures": verdicts.count("failure"), "successes": verdicts.count("success"),
                "stale": verdicts.count("stale"), "other": sum(v not in JUDGED | {"stale"} for v in verdicts),
                "verdicts": "|".join(verdicts), "detected": int("failure" in verdicts), "complete": int(complete),
            }
            self.trials.append(trial)
            out(trial)
            log(f"{self.robot[0]} {self.name} {n}/{len(self.plan)}: tau/SSP {ratio}, {len(rows)} measurement(s), "
                f"{'caught' if trial['detected'] else 'missed'}")
            # The next attack waits for a measurement taken after this restore.
            not_before_ms = t_restore
            last_end = max((row["t_measured_ms"] + row["evidence_ms"] for row in self.rows()), default=0) / 1000 \
                or None
            # Rows of earlier trials are not needed: cleared, so that reading
            # them stays light for the sidecar being measured.
            call(self.robot, "DELETE", "/results")


def point_dir(name):
    return os.path.join(OUT_ROOT, name)


def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--ssp", default="5000,20000", help="SSPs in ms (default: 5000,20000)")
    p.add_argument("--modes", default="fixed,exp", help="measurement schedules among fixed, exp (default: fixed,exp)")
    p.add_argument("--ratios", default="0.05,0.1,0.25,0.5,1,2", help="tau/SSP values (default: 0.05,0.1,0.25,0.5,1,2)")
    p.add_argument("--trials", type=int, default=20, help="trials per ratio and point (default: 20)")
    args = p.parse_args()
    ssps = [int(s) for s in args.ssp.split(",")]
    modes = args.modes.split(",")
    ratios = [float(r) for r in args.ratios.split(",")]
    if not set(modes) <= {"fixed", "exp"}:
        sys.exit(f"--modes must be among fixed, exp (got {args.modes})")
    points = list(itertools.product(ssps, modes))

    if env.get("CONTRACT") != "AttestationManagerV3":
        sys.exit("experiment_toctou.py measures contract V3: deploy it with ./start.sh --contract v3")
    n = int(env.get("N_ROBOTS", 0))
    if env.get("DUMMY") == "TRUE" or env.get("DEPLOY_MODE") == "remote" or n < max(len(points), 4):
        sys.exit(f"experiment_toctou.py needs {max(len(points), 4)} local Gazebo robots (one per point): "
                 f"./start.sh --robots {max(len(points), 4)} --iterq 1")
    with open(os.path.join(ROOT, "blockchain/quorum-test-network/config/besu/genesis.json")) as f:
        block_period = json.load(f)["config"]["qbft"]["blockperiodseconds"]

    robots = [(f"robot{i}", "localhost", ROBOTS_BASE_PORT + i - 1) for i in range(1, n + 1)]
    setup = {
        "contract": env["CONTRACT"], "robots": n, "target": "robot_state_publisher",
        "tampering": "1 byte of 0x90 (NOP) at .text + 0x2000 (tools/tamper.sh --restore-after)",
        "gazebo_world": env.get("GAZEBO_WORLD"), "sidecar_cpu_limit": env.get("CPU_LIMIT"),
        "jury_size": int(env["JURY_SIZE"]), "jury_threshold": int(env["JURY_THRESHOLD"]),
        "freshness_blocks": int(env["FRESHNESS_BLOCKS"]), "block_period_s": block_period, "iterq": 1,
        "ratios": ratios, "trials_per_ratio": args.trials,
    }

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")

    stop = threading.Event()
    jobs = []
    for i, (ssp, mode) in enumerate(points):
        plan = [r for r in ratios for _ in range(args.trials)]
        random.shuffle(plan)
        jobs.append(Point(robots[i], ssp, mode, plan, setup, stop))
    log(f"setup: {n} Gazebo robots; points " + ", ".join(f"{j.robot[0]} {j.name}" for j in jobs)
        + f"; {len(ratios)} ratios x {args.trials} trials each")

    errors = []
    try:
        each(robots + [SECAAS], lambda a: call(a, "GET", "/"))  # all reachable
        stop_robots(robots)
        call(SECAAS, "POST", "/stop", timeout=120)  # a fresh SECaaS loop, kept running
        call(SECAAS, "POST", "/start")
        warm_up(env, robots)
        for i, robot in enumerate(robots):
            ssp, mode = points[i] if i < len(points) else (10000, "fixed")  # spare robots: jurors only
            call(robot, "POST", "/config", json={"rounds": 0, "start_offset_ms": round(random.uniform(0, ssp)),
                                                 "attestation_interval_ms": ssp, "attestation_interval_mode": mode,
                                                 "iterq_threshold": 1})
        each(robots, lambda a: call(a, "DELETE", "/results"))
        start_together(robots)
        time.sleep(max(ssps) / 1000 + 5)  # every robot has measured once

        def run_job(job):
            base = point_dir(job.name)
            os.makedirs(base, exist_ok=True)
            taken = [int(d[3:]) for d in os.listdir(base) if d.startswith("run") and d[3:].isdigit()]
            name = f"run{max(taken, default=0) + 1:02d}"
            tmp = os.path.join(base, f".{name}.tmp")
            os.makedirs(tmp)
            t_start = time.time()
            retries = estimate_retries([job.robot])
            with open(os.path.join(tmp, "trials.csv"), "w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=TRIAL_COLUMNS, extrasaction="ignore")
                w.writeheader()
                def out(trial):
                    w.writerow(trial)
                    f.flush()
                try:
                    job.run(out)
                except Exception as e:
                    errors.append(f"{job.robot[0]} {job.name}: {e}")
                    stop.set()
                    return
            if stop.is_set():
                return
            problems = [f"trial {t['trial']}: the window's rows were not all in time" for t in job.trials
                        if not t["complete"]]
            problems += [f"trial {t['trial']}: {t['other']} measurement(s) neither judged nor stale"
                         for t in job.trials if t["other"]]
            meta = {
                **setup, "robot": job.robot[0], "ssp_ms": job.ssp, "mode": job.mode, "trials": len(job.trials),
                "detected": sum(t["detected"] for t in job.trials),
                "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_start)),
                "duration_s": round(time.time() - t_start), "problems": problems,
                "gas_estimate_retries": estimate_retries([job.robot]) - retries,
                "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                             capture_output=True, text=True).stdout.strip(),
            }
            with open(os.path.join(tmp, "meta.json"), "w") as f:
                json.dump(meta, f, indent=2)
            os.rename(tmp, os.path.join(base, name))
            log(f"{job.robot[0]} {job.name}: {meta['detected']}/{meta['trials']} caught, {len(problems)} problem(s)"
                f" → {os.path.relpath(os.path.join(base, name), ROOT)}")

        # Plain threads: Ctrl-C sets stop, and each point ends after its
        # current attack (whose tamper.sh restores the bytes).
        threads = [threading.Thread(target=run_job, args=(job,), daemon=True) for job in jobs]
        for t in threads:
            t.start()
        try:
            while any(t.is_alive() for t in threads):
                for t in threads:
                    t.join(1)
        except KeyboardInterrupt:
            log("interrupted: stopping after the current attacks")
            stop.set()
            for t in threads:
                t.join()
            raise
    except (KeyboardInterrupt, Exception) as e:
        stop.set()
        errors.append("interrupted" if isinstance(e, KeyboardInterrupt) else repr(e))
    finally:
        stop.set()
        try:
            stop_robots(robots)
        except requests.RequestException:
            pass
    if errors:
        sys.exit("stopped: " + "; ".join(errors) + " (the unfinished points stay in their .runNN.tmp/)")
    log("done")


if __name__ == "__main__":
    main()
