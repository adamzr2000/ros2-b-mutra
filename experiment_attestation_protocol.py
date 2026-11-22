#!/usr/bin/env python3
"""Attestation protocol experiment (contract V3): latency of both phases,
and resources of the sidecars and the SECaaS.

Terms used by every experiment:
  setup       what ./start.sh deploys: robots, dummy or Gazebo, local or remote
              hosts, contract, jury, sidecar CPU cap, freshness window.
  experiment  a script experiment_<name>.py that measures one thing on a
              setup. Its data goes to experiments/data/<name>/ (this one:
              attestation-protocol).
  condition   one combination of parameters: phase, N robots, SSP, IterQ K.
  run         one repetition of a condition.
  round       one attestation, from the evidence to the verdict.

Phases:
  bootstrap   the chain is reset, so no robot is eligible as a juror; all
              robots start together and send one round each, judged by the
              SECaaS. A Prove that misses the first block finds robots
              already eligible and may be judged by a jury (closed_by in
              rounds.csv).
  continuous  all robots are eligible; all start together, each prover after
              a random offset within one SSP, and each sends --rounds rounds,
              judged by juries.
  both        per run: a bootstrap run, then a continuous run.

The robots and their hosts come from the setup (.env). Before the first run,
one bootstrap run is made and not recorded: it warms up the agents and checks
that every robot attests successfully. The bootstrap runs of an experiment
start at evenly spaced points of the block period (block_phase_ms: after a
block, run i of R at (i - 0.5) / R of it); continuous provers start at a
random offset within one SSP.

Runs are appended to their condition folder (runNN numbering continues) only
when the folder's runs have the same setup. Each run's data is checked
(rounds, judges, verifier rows, transactions, blocks, resource samples); what
does not match is listed under "problems" in meta.json. One experiment runs at
a time; Ctrl-C stops the robots and the run in progress is not written.

Each run is written to experiments/data/attestation-protocol/<phase>/N<n>-SSP<ms>-K<k>/runNN/:
  meta.json      setup, parameters, block range, start offsets
  prover.csv     sidecars: one row per round sent, its steps and total
  verifier.csv   sidecars and SECaaS: one row per round each was asked to
                 judge (juror or SECaaS verdict, candidate not drawn), its
                 steps and total
  resources.csv  resource collectors: one row per second per sidecar and the SECaaS
  rounds.csv     chain: one row per round, its blocks, judge and verdict
Gas and block contents are measured by experiment_blockchain_impact.py.

Durations are measured inside one process (monotonic clock), so they hold
across hosts without clock synchronization; t_*_ms columns only order rows.
The agents learn of each new block from their node's WebSocket notification
(eth_subscribe newHeads), so the block steps are timed to a few ms; the
experiment refuses to run while an agent falls back to polling.

Example, after ./start.sh --dummy --robots 4:
  python3 experiment_attestation_protocol.py --phase both --runs 20 --rounds 10
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
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "attestation-protocol")
LOCK = os.path.join(ROOT, "experiments", "data", ".experiment.lock")  # one experiment at a time on the stack
RPC_URL = "http://localhost:21001"  # validator1
SECAAS = ("secaas", "localhost", 8000)
ROBOTS_BASE_PORT = 8001
REMOTE1_LIMIT = 64  # remote setup: robots 1..64 on REMOTE1_HOST, the others on REMOTE2_HOST
COLLECTOR_PORT = 6000  # resource collector, one per host
ADMIN_KEYS = [os.path.join(ROOT, "blockchain/quorum-test-network/config/nodes", v, "accountPrivateKey")
              for v in ("validator2", "validator3")]

PROVER_COLUMNS = ["round_id", "robot", "t_sent_ms", "evidence_ms", "presend_ms", "send_ms", "inclusion_ms",
                  "judgement_ms", "total_ms", "outcome"]
VERIFIER_COLUMNS = ["round_id", "verifier", "role", "t_detected_ms", "detect_ms", "queue_ms", "check_ms",
                    "read_ms", "compute_ms", "send_ms", "total_ms", "attempts", "verdict", "outcome"]
ROUND_COLUMNS = ["round_id", "prover", "prove_block", "close_block", "blocks_to_close", "closed_by", "verdict",
                 "votes"]
RESOURCE_COLUMNS = ["t_ms", "container", "cpu_percent", "mem_mb", "net_rx_kb", "net_tx_kb"]

# One session for every call: its connections stay open. Each agent is its
# own host:port, so the pool keeps one per agent (up to 512).
HTTP = requests.Session()
HTTP.mount("http://", requests.adapters.HTTPAdapter(pool_connections=512, pool_maxsize=8))

BOOTSTRAP_FAILED = ("bootstrap failed: not every robot is eligible. If the sidecar image was rebuilt, "
                    "its references changed: run ./start.sh again.")


# ── setup ─────────────────────────────────────────────────────────────────────

def read_env():
    env = {}
    with open(os.path.join(ROOT, ".env")) as f:
        for line in f:
            key, sep, val = line.strip().partition("=")
            if sep and not key.startswith("#"):
                env[key.strip()] = val.strip()
    return env


def address_names(dummy):
    """{lowercase address: robotN or secaas}, from the agent configs."""
    names = {}
    config_dir = os.path.join(ROOT, "config-dummy" if dummy else "config")
    for name in os.listdir(config_dir):
        if name.startswith("robot") and name.endswith(".json"):
            with open(os.path.join(config_dir, name)) as f:
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


def wait_new_block():
    """Returns within ~20 ms of the node importing a new block, with the
    time.monotonic() it saw it."""
    start = head()
    while head() == start:
        time.sleep(0.02)
    return time.monotonic()


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
    wait_blocks(2)  # the host settles after the reset tooling


# ── agents ────────────────────────────────────────────────────────────────────

def deployed_robots(env):
    """[(name, host, port)] of the robots in the setup."""
    n = int(env["N_ROBOTS"])
    if env.get("DEPLOY_MODE") != "remote":
        hosts = ["localhost"] * n
    else:
        hosts = [env["REMOTE1_HOST"] if i <= REMOTE1_LIMIT else env["REMOTE2_HOST"] for i in range(1, n + 1)]
    return [(f"robot{i}", hosts[i - 1], ROBOTS_BASE_PORT + i - 1) for i in range(1, n + 1)]


def url(agent, path):
    _, host, port = agent
    return f"http://{host}:{port}{path}"


def call(agent, method, path, **kw):
    r = HTTP.request(method, url(agent, path), timeout=kw.pop("timeout", 10), **kw)
    r.raise_for_status()
    return r.json()


def each(agents, fn):
    with ThreadPoolExecutor(max_workers=max(1, len(agents))) as pool:
        return list(pool.map(fn, agents))


def wait_finished(robots, timeout):
    deadline = time.time() + timeout
    while time.time() < deadline:
        pending = [a[0] for a, s in zip(robots, each(robots, lambda a: call(a, "GET", "/status")["status"]))
                   if s != "finished"]
        if not pending:
            return True
        time.sleep(1)
    print(f"   ⚠️  not finished after {timeout:.0f}s: {', '.join(pending)}")
    return False


def start_together(agents):
    """POST /start to all agents at the same instant."""
    barrier = threading.Barrier(len(agents))
    def start(agent):
        barrier.wait()
        call(agent, "POST", "/start")
    each(agents, start)


def estimate_retries(agents):
    """Gas estimates the agents tried again after a Besu "Internal error"
    (each adds ~100 ms to a step): recorded per run, to flag runs if needed."""
    return sum(each(agents, lambda a: call(a, "GET", "/status")["counters"].get("gas_estimate_retries", 0)))


def collectors(robots):
    """{collector URL: [containers on its host]}: the sidecars, and the SECaaS here."""
    groups = {f"http://localhost:{COLLECTOR_PORT}": ["secaas"]}
    for name, host, _ in robots:
        groups.setdefault(f"http://{host}:{COLLECTOR_PORT}", []).append(f"{name}-sidecar")
    return groups


# ── one run ───────────────────────────────────────────────────────────────────

# meta.json keys that must be equal for all the runs of a condition folder.
SETUP_KEYS = ["contract", "jury_size", "jury_threshold", "jury_timeout_blocks", "freshness_blocks",
              "block_period_s", "sidecar_cpu_limit", "dummy", "deploy_mode", "hosts", "ssp_mode",
              "rounds_per_robot"]


def condition_dir(phase, n, args):
    return os.path.join(OUT_ROOT, phase, f"N{n}-SSP{args.ssp}-K{args.iterq}")


def check_condition(base, setup):
    """Exits if the runs already in base were made with another setup."""
    if not os.path.isdir(base):
        return
    for d in sorted(os.listdir(base)):
        meta_path = os.path.join(base, d, "meta.json")
        if not os.path.isfile(meta_path):
            continue
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


def check_run(phase, setup, n, rounds, first, last, prover, verifier, resources, data, containers):
    """Consistency of the data of one run: [problem], empty when consistent."""
    problems = []
    ids = {r["round_id"] for r in prover}
    if len(prover) != n * rounds:
        problems.append(f"{len(prover)} prover rows, expected {n * rounds}")
    bad = sorted({r["outcome"] for r in prover} - {"success"})
    if bad:
        problems.append(f"prover outcomes {bad}")

    chain_rounds = {r["round_id"]: r for r in data["rounds"]}
    if set(chain_rounds) != ids:
        problems.append(f"rounds on chain and in prover.csv differ: {len(set(chain_rounds) ^ ids)} round(s)")
    # Bootstrap: the SECaaS judges the rounds of the first block. A Prove
    # that misses it finds robots already eligible and may get a jury.
    first_prove = min((r["prove_block"] for r in chain_rounds.values()), default=0)
    for r in chain_rounds.values():
        if phase == "continuous":
            judges = {"jury"}
        else:
            judges = {"secaas"} if r["prove_block"] == first_prove else {"secaas", "jury"}
        if r["closed_by"] not in judges or r["verdict"] != "success":
            problems.append(f"round {r['round_id'][-8:]} closed by {r['closed_by'] or 'nobody'}, "
                            f"verdict {r['verdict'] or 'none'}")
        elif r["closed_by"] == "jury" and r["votes"] != setup["jury_threshold"]:
            problems.append(f"round {r['round_id'][-8:]} closed with {r['votes']} votes")
        if r["gas_reverted"]:
            problems.append(f"round {r['round_id'][-8:]} has reverted transactions")

    # A SECaaS round has one SECaaS verdict; a jury round the drawn jurors'
    # verdicts, and in continuous runs a check from each other robot.
    sent, rows = {}, {}
    for v in verifier:
        rows[v["round_id"]] = rows.get(v["round_id"], 0) + 1
        if v["role"] in ("secaas", "juror") and v["outcome"] == "sent":
            sent[v["round_id"]] = sent.get(v["round_id"], 0) + 1
        elif not (v["role"] == "candidate" and v["outcome"] in ("not_drawn", "late")):
            problems.append(f"{v['verifier']} {v['role']} {v['outcome']} on round {v['round_id'][-8:]}")
    for rid in ids:
        jury = chain_rounds.get(rid, {}).get("closed_by") == "jury"
        expected = setup["jury_threshold"] if jury else 1
        if sent.get(rid, 0) != expected:
            problems.append(f"round {rid[-8:]}: {sent.get(rid, 0)} verdict rows, expected {expected}")
        if phase == "continuous" and rows.get(rid, 0) != n - 1:
            problems.append(f"round {rid[-8:]}: {rows.get(rid, 0)} verifier rows, expected {n - 1}")

    expected = {"Prove", "VerifyBySECaaS", "Vote"} if phase == "bootstrap" else {"Prove", "Vote"}
    other = sorted({t["function"] for t in data["transactions"]} - expected)
    if other:
        problems.append(f"other transactions to the contract during the run: {other}")

    blocks = [b["block"] for b in data["blocks"]]
    if blocks != list(range(first, last + 1)):
        problems.append("the chain data does not cover the run's block range")
    sampled = {r["container"] for r in resources}
    if sampled != set(containers):
        problems.append(f"no resource samples for {sorted(set(containers) - sampled)}")
    return problems


def run(phase, args, setup, env, robots, names, record=True, slot=(0, 1)):
    """One run of a phase. Returns whether every round succeeded. slot is
    (i, of): bootstrap run i of this experiment's of runs, i from 0."""
    n, ssp, rounds = len(robots), args.ssp, (1 if phase == "bootstrap" else args.rounds)
    if phase == "bootstrap":
        print("   resetting the chain: no robot eligible")
        reset_chain(env["CONTRACT"])
        # All robots start at the same moment, so a run sends at one point of
        # the block period: the runs start at evenly spaced points of it.
        block_phase_ms = round((slot[0] + 0.5) / slot[1] * args.block_period * 1000)
        offsets = [0] * n
    else:
        block_phase_ms = None
        offsets = [round(random.uniform(0, ssp)) for _ in range(n)]

    # Every juror runs from the start; only the provers wait their offset.
    each(list(zip(robots, offsets)), lambda ao: call(ao[0], "POST", "/config", json={
        "rounds": rounds, "start_offset_ms": ao[1], "attestation_interval_ms": ssp,
        "attestation_interval_mode": setup["ssp_mode"], "iterq_threshold": args.iterq}))
    each(robots + [SECAAS], lambda a: call(a, "DELETE", "/results"))
    retries = estimate_retries(robots)
    groups = collectors(robots)
    for collector, containers in groups.items():
        HTTP.post(f"{collector}/start", json={"containers": containers}, timeout=10).raise_for_status()
    if block_phase_ms is not None:
        wait_new_block()
        time.sleep(block_phase_ms / 1000)
    first = head() + 1
    t_start = time.time()
    start_together(robots)
    timeout = max(offsets) / 1000 + rounds * (args.iterq * ssp / 1000 + 30) + 60
    wait_finished(robots, timeout)
    wait_blocks(2)  # a late vote goes in the next block
    each(robots, lambda a: call(a, "POST", "/stop", timeout=30))
    last = head()
    t_end = time.time()
    resources = []
    for collector in groups:
        HTTP.post(f"{collector}/stop", timeout=10).raise_for_status()
        resources += HTTP.get(f"{collector}/results", timeout=30).json()["samples"]

    prover = [r for rows in each(robots, lambda a: call(a, "GET", "/results")["prover"]) for r in rows]
    verifier = [r for rows in each(robots + [SECAAS], lambda a: call(a, "GET", "/results")["verifier"]) for r in rows]
    data = json.loads(chain("extract", first, last))
    for r in data["rounds"]:
        r["prover"] = names.get(r["prover"].lower(), r["prover"])
    for t in data["transactions"]:
        t["sender"] = names.get(t["sender"].lower(), t["sender"])

    ok = len(prover) == n * rounds and all(r["outcome"] == "success" for r in prover)
    problems = check_run(phase, setup, n, rounds, first, last, prover, verifier, resources, data,
                         [c for cs in groups.values() for c in cs])
    closed = {}
    for r in data["rounds"]:
        closed[r["closed_by"] or "open"] = closed.get(r["closed_by"] or "open", 0) + 1
    print(f"   {len(prover)}/{n * rounds} rounds, closed by {closed}, blocks {first}..{last}, "
          f"{len(problems)} problem(s)")
    for problem in problems[:10]:
        print(f"   ⚠️  {problem}")
    if not record:
        return ok

    meta = {
        "phase": phase, "robots": n, "ssp_ms": ssp, "iterq": args.iterq, **setup, "rounds_per_robot": rounds,
        "start_offsets_ms": dict(zip((a[0] for a in robots), offsets)), "block_phase_ms": block_phase_ms,
        "first_block": first, "last_block": last,
        "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t_start)),
        "duration_s": round(t_end - t_start, 1), "all_success": ok, "problems": problems,
        "gas_estimate_retries": estimate_retries(robots) - retries,
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }
    path = write_run(condition_dir(phase, n, args), {
        "prover.csv": (PROVER_COLUMNS, sorted(prover, key=lambda r: r.get("t_sent_ms", 0))),
        "verifier.csv": (VERIFIER_COLUMNS, sorted(verifier, key=lambda r: r["t_detected_ms"])),
        "resources.csv": (RESOURCE_COLUMNS, sorted(resources, key=lambda r: r["t_ms"])),
        "rounds.csv": (ROUND_COLUMNS, data["rounds"]),
    }, meta)
    print(f"   → {os.path.relpath(path, ROOT)}")
    return ok


# ── main ──────────────────────────────────────────────────────────────────────

def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--phase", choices=["bootstrap", "continuous", "both"], required=True)
    p.add_argument("--runs", type=int, default=1, help="runs per phase (default: 1)")
    p.add_argument("--rounds", type=int, default=10, help="continuous: rounds per robot (default: 10)")
    p.add_argument("--ssp", type=int, default=int(env.get("ATTESTATION_INTERVAL_MS", 10000)),
                   help="ms between measurements (default: ATTESTATION_INTERVAL_MS of .env)")
    p.add_argument("--iterq", type=int, default=int(env.get("ITERQ_THRESHOLD", 1)),
                   help="measurements per round (default: ITERQ_THRESHOLD of .env)")
    args = p.parse_args()

    if env.get("CONTRACT") != "AttestationManagerV3":
        sys.exit("experiment_attestation_protocol.py measures contract V3: deploy it with ./start.sh --contract v3")
    with open(os.path.join(ROOT, "blockchain/quorum-test-network/config/besu/genesis.json")) as f:
        args.block_period = json.load(f)["config"]["qbft"]["blockperiodseconds"]
    # The window was sized at deployment for the SSP and IterQ of .env.
    needed = math.ceil(2 * (args.iterq - 1) * args.ssp / (args.block_period * 1000)) + 15
    if needed > int(env["FRESHNESS_BLOCKS"]):
        sys.exit(f"--ssp {args.ssp} with --iterq {args.iterq} needs a freshness window of {needed} blocks, "
                 f"the contract has {env['FRESHNESS_BLOCKS']}: redeploy with ./start.sh --ssp/--iterq")

    robots = deployed_robots(env)
    names = address_names(env.get("DUMMY") == "TRUE")
    setup = {
        "contract": env["CONTRACT"], "jury_size": int(env["JURY_SIZE"]), "jury_threshold": int(env["JURY_THRESHOLD"]),
        "jury_timeout_blocks": int(env["JURY_TIMEOUT_BLOCKS"]), "freshness_blocks": int(env["FRESHNESS_BLOCKS"]),
        "block_period_s": args.block_period, "sidecar_cpu_limit": env.get("CPU_LIMIT"),
        "dummy": env.get("DUMMY") == "TRUE", "deploy_mode": env.get("DEPLOY_MODE"),
        "hosts": sorted({a[1] for a in robots}), "ssp_mode": env.get("ATTESTATION_INTERVAL_MODE", "fixed"),
    }
    phases = ["bootstrap", "continuous"] if args.phase == "both" else [args.phase]
    for phase in phases:
        check_condition(condition_dir(phase, len(robots), args),
                        {**setup, "rounds_per_robot": 1 if phase == "bootstrap" else args.rounds})

    print(f"setup: {len(robots)} robots on {', '.join(sorted({a[1] for a in robots}))}, SSP {args.ssp} ms, IterQ {args.iterq}, jury "
          f"{env['JURY_THRESHOLD']}/{env['JURY_SIZE']}, block period {args.block_period} s")
    modes = each(robots, lambda a: call(a, "GET", "/status").get("block_notifications"))
    modes.append(call(SECAAS, "GET", "/").get("block_notifications"))
    polling = [a[0] for a, m in zip(robots + [SECAAS], modes) if m != "websocket"]
    if polling:
        sys.exit(f"no WebSocket block notifications at {', '.join(polling)}: the block steps would be "
                 "polled. Check the validators' WebSocket ports (2200x) and the agents' eth_ws_url, "
                 "then ./start.sh again")

    lock = open(LOCK, "w")
    try:
        fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
    except BlockingIOError:
        sys.exit("another experiment is running on this stack")
    try:
        experiment(args, setup, env, robots, names, phases)
    except KeyboardInterrupt:
        print("\ninterrupted: the run in progress is not written; stopping the robots")
        stop_all(robots)
        sys.exit(130)


def stop_all(robots):
    for collector in collectors(robots):
        HTTP.post(f"{collector}/stop", timeout=10).raise_for_status()
    each(robots, lambda a: call(a, "POST", "/stop", timeout=30))


def experiment(args, setup, env, robots, names, phases):
    stop_all(robots)
    call(SECAAS, "POST", "/stop", timeout=120)  # a fresh SECaaS loop, kept running
    call(SECAAS, "POST", "/start")

    print("── warm-up: bootstrap, not recorded")
    if not run("bootstrap", args, setup, env, robots, names, record=False) or int(chain("eligible")) < len(robots):
        sys.exit(BOOTSTRAP_FAILED)

    for i in range(1, args.runs + 1):
        for phase in phases:
            print(f"── run {i}/{args.runs}: {phase}")
            if not run(phase, args, setup, env, robots, names, slot=(i - 1, args.runs)) and phase == "bootstrap":
                sys.exit(BOOTSTRAP_FAILED)


if __name__ == "__main__":
    main()
