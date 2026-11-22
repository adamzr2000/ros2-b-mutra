#!/usr/bin/env python3
"""Blockchain impact experiment (contract V3): what attestation adds to the
ledger and to the validators, by number of robots N and IterQ K, at a fixed
SSP. The data answers how much the ledger and the service store per day,
month and year.

Setup: dummy robots; the freshness window sized for the largest K, e.g.
  ./start.sh --dummy --robots 4 --ssp 10000 --iterq 10

Conditions, in a new random order in each run:
  idle  the robots do not attest: the chain's own growth (QBFT seals a block
        every period, empty or not)
  K<k>  every robot attests continuously, K measurements per round, one
        round about every K x SSP; provers start at random offsets within
        one cycle. Recording starts once every robot has sent a round.
Each condition is recorded for --window seconds, then the robots stop and
the chain rests for --cooldown seconds: a round left open by the stop
expires to the SECaaS meanwhile. Every condition starts with all robots
eligible as jurors: before the first one, and whenever an expired round cost
its jurors their eligibility, the chain is reset and one round per robot is
attested, not recorded.

Each condition of each run is written to
experiments/data/blockchain-impact/N<n>-SSP<ms>-<condition>/runNN/:
  meta.json         setup, condition, block range, the validators' database
                    size and the SECaaS database size at the window's start
                    and end, rounds
  blocks.csv        one row per block of the window: bytes, gas, transactions
  transactions.csv  one row per transaction to the contract: function, gas,
                    calldata, signed and log bytes
  rounds.csv        one row per round proved in the window: blocks, judge,
                    verdict, gas
  validators.csv    one row per second per validator: CPU, memory, network
Runs are appended (runNN numbering continues) only when the folder's runs
have the same setup. What a check finds is listed under "problems" in
meta.json. One experiment runs at a time; Ctrl-C stops the robots and the
condition in progress is not written.

Example:
  python3 experiment_blockchain_impact.py
"""

import argparse
import csv
import fcntl
import json
import math
import os
import random
import subprocess
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor

import requests

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "blockchain-impact")
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
RPC_URL = "http://localhost:21001"  # validator1
SECAAS = ("secaas", "localhost", 8000)
RESOURCES = "http://localhost:6000"  # the validators run on this host
VALIDATORS = [f"quorum-test-network-validator{i}-1" for i in range(1, 5)]
ROBOTS_BASE_PORT = 8001
REMOTE1_LIMIT = 64
ADMIN_KEYS = [os.path.join(ROOT, "blockchain/quorum-test-network/config/nodes", v, "accountPrivateKey")
              for v in ("validator2", "validator3")]

BLOCK_COLUMNS = ["block", "timestamp", "tx_count", "gas_used", "gas_limit", "size_bytes"]
TX_COLUMNS = ["block", "sender", "function", "round_id", "gas_used", "status", "calldata_bytes", "tx_bytes",
              "log_bytes", "events"]
ROUND_COLUMNS = ["round_id", "prover", "prove_block", "close_block", "blocks_to_close", "closed_by", "verdict",
                 "votes", "gas_prove", "gas_votes", "gas_secaas", "gas_expire", "gas_reverted"]
RESOURCE_COLUMNS = ["t_ms", "container", "cpu_percent", "mem_mb", "net_rx_kb", "net_tx_kb"]
# meta.json keys that must be equal for all the runs of a condition folder.
SETUP_KEYS = ["contract", "robots", "ssp_ms", "jury_size", "jury_threshold", "jury_timeout_blocks",
              "freshness_blocks", "block_period_s", "sidecar_cpu_limit", "deploy_mode", "hosts", "window_s"]

# One session for every call; each agent is its own host:port.
HTTP = requests.Session()
HTTP.mount("http://", requests.adapters.HTTPAdapter(pool_connections=512, pool_maxsize=8))


# ── setup ─────────────────────────────────────────────────────────────────────

def read_env():
    env = {}
    with open(os.path.join(ROOT, ".env")) as f:
        for line in f:
            key, sep, val = line.strip().partition("=")
            if sep and not key.startswith("#"):
                env[key.strip()] = val.strip()
    return env


def deployed_robots(env):
    """[(name, host, port)] of the robots in the setup."""
    n = int(env["N_ROBOTS"])
    if env.get("DEPLOY_MODE") != "remote":
        hosts = ["localhost"] * n
    else:
        hosts = [env["REMOTE1_HOST"] if i <= REMOTE1_LIMIT else env["REMOTE2_HOST"] for i in range(1, n + 1)]
    return [(f"robot{i}", hosts[i - 1], ROBOTS_BASE_PORT + i - 1) for i in range(1, n + 1)]


def address_names():
    """{lowercase address: robotN or secaas}, from the agent configs."""
    names = {}
    for name in os.listdir(os.path.join(ROOT, "config-dummy")):
        if name.startswith("robot") and name.endswith(".json"):
            with open(os.path.join(ROOT, "config-dummy", name)) as f:
                names[json.load(f)["eth_address"].lower()] = name[:-5]
    with open(os.path.join(ROOT, "config", "secaas.json")) as f:
        names[json.load(f)["eth_address"].lower()] = "secaas"
    return names


def rpc(method, params=()):
    r = HTTP.post(RPC_URL, json={"jsonrpc": "2.0", "id": 1, "method": method, "params": list(params)}, timeout=10)
    r.raise_for_status()
    return r.json()["result"]


def head():
    return int(rpc("eth_blockNumber"), 16)


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


def disk_bytes():
    """{validator: bytes of its database directory}."""
    def du(v):
        out = subprocess.run(["docker", "exec", v, "du", "-sb", "/opt/besu/data"], capture_output=True, text=True)
        return int(out.stdout.split()[0]) if out.returncode == 0 else None
    return dict(zip(VALIDATORS, each(VALIDATORS, du)))


def secaas_db_bytes():
    out = subprocess.run(["docker", "exec", "attestation-db", "psql", "-U", "postgres", "-d", "attestation", "-tAc",
                          "select pg_database_size('attestation')"], capture_output=True, text=True)
    return int(out.stdout.strip()) if out.returncode == 0 and out.stdout.strip().isdigit() else None


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


def configure(robots, rounds, ssp, k, offsets):
    each(list(zip(robots, offsets)), lambda ro: call(ro[0], "POST", "/config", json={
        "rounds": rounds, "start_offset_ms": ro[1], "attestation_interval_ms": ssp,
        "attestation_interval_mode": "fixed", "iterq_threshold": k}))


def warm_up(env, robots):
    """One round per robot after a chain reset: every robot then eligible."""
    print("   warm-up: chain reset, one round per robot, not recorded")
    reset_chain(env["CONTRACT"])
    configure(robots, 1, 10000, 1, [0] * len(robots))
    start_together(robots)
    deadline = time.time() + 120 + 0.1 * len(robots)
    while time.time() < deadline:
        if all(s == "finished" for s in each(robots, lambda a: call(a, "GET", "/status")["status"])):
            break
        time.sleep(2)
    stop_robots(robots)
    wait_blocks(2)
    eligible = int(chain("eligible"))
    if eligible < len(robots):
        sys.exit(f"warm-up: {eligible} of {len(robots)} robots eligible. If the sidecar image was rebuilt, its "
                 "references changed: run ./start.sh again.")


# ── one condition ─────────────────────────────────────────────────────────────

def condition_dir(n, ssp, name):
    return os.path.join(OUT_ROOT, f"N{n}-SSP{ssp}-{name}")


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


def run_condition(name, k, args, setup, env, robots, names):
    n, ssp = len(robots), args.ssp
    if k and int(chain("eligible")) < n:
        warm_up(env, robots)
    offsets = [0] * n
    if k:
        cycle_ms = k * ssp
        offsets = [round(random.uniform(0, cycle_ms)) for _ in range(n)]
        configure(robots, 0, ssp, k, offsets)
        start_together(robots)
        # Steady once every robot has sent its first round: its offset plus
        # the K-1 waits of its first batch, then the block that includes it.
        settle = max(offsets) / 1000 + (k - 1) * ssp / 1000 + 2 * setup["block_period_s"]
    else:
        settle = args.cooldown
    time.sleep(settle)

    disk_start, db_start = disk_bytes(), secaas_db_bytes()
    retries = estimate_retries(robots)
    HTTP.post(f"{RESOURCES}/start", json={"containers": VALIDATORS}, timeout=10).raise_for_status()
    first = head() + 1
    t_start = time.time()
    time.sleep(args.window)
    last = head()
    window_s = time.time() - t_start
    HTTP.post(f"{RESOURCES}/stop", timeout=10).raise_for_status()
    disk_end, db_end = disk_bytes(), secaas_db_bytes()
    resources = HTTP.get(f"{RESOURCES}/results", timeout=30).json()["samples"]
    if k:
        stop_robots(robots)
    # The rounds proved near the end close a block later: read a few more.
    wait_blocks(3)
    data = json.loads(chain("extract", first, last + 3))
    blocks = [b for b in data["blocks"] if b["block"] <= last]
    txs = [t for t in data["transactions"] if t["block"] <= last]
    rounds = [r for r in data["rounds"] if r["prove_block"] <= last]
    for t in txs:
        t["sender"] = names.get(t["sender"].lower(), t["sender"])
    for r in rounds:
        r["prover"] = names.get(r["prover"].lower(), r["prover"])
    time.sleep(args.cooldown)

    problems = []
    if [b["block"] for b in blocks] != list(range(first, last + 1)):
        problems.append("the chain data does not cover the window's blocks")
    if {r["container"] for r in resources} != set(VALIDATORS):
        problems.append(f"resource samples for {sorted({r['container'] for r in resources})}")
    if None in disk_start.values() or None in disk_end.values():
        problems.append("a validator's database size could not be read")
    if k:
        expected = n * window_s * 1000 / (k * ssp)
        if not 0.6 * expected <= len(rounds) <= 1.4 * expected:
            problems.append(f"{len(rounds)} rounds, expected about {expected:.0f}")
        # The jurors of a round proved in the last blocks may stop before
        # voting: it then expires to the SECaaS, after the window.
        settled = [r for r in rounds if r["prove_block"] <= last - 2]
        bad = [r for r in settled if r["closed_by"] != "jury" or r["verdict"] != "success" or r["gas_reverted"]]
        if bad:
            problems.append(f"{len(bad)} round(s) not closed by a jury with success, or with reverted transactions")
        other = sorted({t["function"] for t in txs} - {"Prove", "Vote"})
        if other:
            problems.append(f"other transactions to the contract: {other}")
    elif txs:
        problems.append(f"{len(txs)} transactions to the contract while idle")

    block_bytes = sum(b["size_bytes"] for b in blocks)
    print(f"   {name:5s} blocks {first}..{last}, {len(rounds)} rounds, {len(txs)} txs, {block_bytes / 1024:.0f} kB "
          f"of blocks, validator1 database +{(disk_end[VALIDATORS[0]] or 0) - (disk_start[VALIDATORS[0]] or 0)} B, "
          f"{len(problems)} problem(s)")
    for p in problems:
        print(f"   ⚠️  {p}")

    meta = {
        **setup, "condition": name, "iterq": k, "start_offsets_ms": dict(zip((a[0] for a in robots), offsets)),
        "settle_s": round(settle, 1), "first_block": first, "last_block": last, "window_measured_s": round(window_s, 2),
        "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_start)),
        "validator_db_bytes_start": disk_start, "validator_db_bytes_end": disk_end,
        "secaas_db_bytes_start": db_start, "secaas_db_bytes_end": db_end,
        "rounds": len(rounds), "problems": problems,
        "gas_estimate_retries": estimate_retries(robots) - retries,
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }
    path = write_run(condition_dir(n, ssp, name), {
        "blocks.csv": (BLOCK_COLUMNS, blocks),
        "transactions.csv": (TX_COLUMNS, txs),
        "rounds.csv": (ROUND_COLUMNS, rounds),
        "validators.csv": (RESOURCE_COLUMNS, sorted(resources, key=lambda r: r["t_ms"])),
    }, meta)
    print(f"   → {os.path.relpath(path, ROOT)}")


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--runs", type=int, default=1, help="runs; each one records every condition (default: 1)")
    p.add_argument("--iterq", default="1,5,10", help="K values (default: 1,5,10)")
    p.add_argument("--ssp", type=int, default=10000, help="ms between measurements (default: 10000)")
    p.add_argument("--window", type=int, default=600, help="seconds recorded per condition (default: 600)")
    p.add_argument("--cooldown", type=int, default=10,
                   help="seconds of rest after each condition, at least the jury timeout plus 5 blocks (default: 10)")
    args = p.parse_args()
    ks = [int(k) for k in args.iterq.split(",")]

    if env.get("CONTRACT") != "AttestationManagerV3":
        sys.exit("experiment_blockchain_impact.py measures contract V3: deploy it with ./start.sh --contract v3")
    if env.get("DUMMY") != "TRUE":
        sys.exit("experiment_blockchain_impact.py runs dummy robots: ./start.sh --dummy --robots N "
                 f"--ssp {args.ssp} --iterq {max(ks)}")
    with open(os.path.join(ROOT, "blockchain/quorum-test-network/config/besu/genesis.json")) as f:
        block_period = json.load(f)["config"]["qbft"]["blockperiodseconds"]
    needed = math.ceil(2 * (max(ks) - 1) * args.ssp / (block_period * 1000)) + 15
    if needed > int(env["FRESHNESS_BLOCKS"]):
        sys.exit(f"K={max(ks)} at SSP {args.ssp} ms needs a freshness window of {needed} blocks, the contract has "
                 f"{env['FRESHNESS_BLOCKS']}: ./start.sh --dummy --robots {env['N_ROBOTS']} --ssp {args.ssp} "
                 f"--iterq {max(ks)}")

    # A round left open by the stop expires to the SECaaS after the jury
    # timeout (Expire, then its verdict): the rest outlasts both.
    args.cooldown = max(args.cooldown, (int(env["JURY_TIMEOUT_BLOCKS"]) + 5) * block_period)
    robots = deployed_robots(env)
    names = address_names()
    setup = {
        "contract": env["CONTRACT"], "robots": len(robots), "ssp_ms": args.ssp,
        "jury_size": int(env["JURY_SIZE"]), "jury_threshold": int(env["JURY_THRESHOLD"]),
        "jury_timeout_blocks": int(env["JURY_TIMEOUT_BLOCKS"]), "freshness_blocks": int(env["FRESHNESS_BLOCKS"]),
        "block_period_s": block_period, "sidecar_cpu_limit": env.get("CPU_LIMIT"),
        "deploy_mode": env.get("DEPLOY_MODE"), "hosts": sorted({a[1] for a in robots}), "window_s": args.window,
        "cooldown_s": args.cooldown,
    }
    conditions = [("idle", None)] + [(f"K{k}", k) for k in ks]
    for name, _ in conditions:
        check_condition(condition_dir(len(robots), args.ssp, name), setup)

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")

    print(f"setup: {len(robots)} robots on {', '.join(setup['hosts'])}, SSP {args.ssp} ms, K {ks}, "
          f"{args.runs} run(s) of {args.window} s per condition")
    try:
        each(robots + [SECAAS], lambda a: call(a, "GET", "/"))  # all reachable
        HTTP.post(f"{RESOURCES}/stop", timeout=10).raise_for_status()
        stop_robots(robots)
        call(SECAAS, "POST", "/stop", timeout=120)  # a fresh SECaaS loop, kept running
        call(SECAAS, "POST", "/start")
        for i in range(1, args.runs + 1):
            order = random.sample(conditions, len(conditions))
            print(f"── run {i}/{args.runs}: {', '.join(n for n, _ in order)}")
            for name, k in order:
                run_condition(name, k, args, setup, env, robots, names)
    except KeyboardInterrupt:
        print("\ninterrupted: the condition in progress is not written; stopping the robots")
        try:
            HTTP.post(f"{RESOURCES}/stop", timeout=10)
        except requests.RequestException:
            pass
        stop_robots(robots)
        sys.exit(130)


if __name__ == "__main__":
    main()
