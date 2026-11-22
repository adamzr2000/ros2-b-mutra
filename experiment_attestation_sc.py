#!/usr/bin/env python3
"""Smart contract experiment (contract V3): the gas of the functions the
protocol relies on, on each of their paths, by number of agents.

Runs smart-contracts/scripts/characterize_gas.js in the hardhat container, on
Hardhat's in-memory network: London rules, as the Besu genesis, so the gas
equals the gas on the chain (gas is deterministic). Calldata is shaped as the
agents send it. No running stack is needed.

Measured: per round, Prove, Vote, VerifyBySECaaS and Expire on every path
(normal jury round, bootstrap by the SECaaS, failure verdict, dissent and
expiry handed to the SECaaS); per robot onboarding, RegisterAgent and the
admins' SetReference approvals. For each number of agents in --agents, on a
fresh contract; then the jury draw with only a fraction of the agents
eligible (--eligible, on --fraction-agents agents).

The jury settings default to the deployed ones (.env), else 2 of 2, 5-block
timeout, 15-block freshness window.

Written to experiments/data/attestation-sc/J<size>-T<threshold>/runNN/:
  meta.json  settings, compiler, hardfork, agents, rounds
  gas.csv    one row per transaction: agents, eligible_before, scenario,
             function, path, gas, calldata_bytes, log_bytes

Example:
  python3 experiment_attestation_sc.py
"""

import argparse
import csv
import hashlib
import json
import os
import subprocess
import sys
import time

ROOT = os.path.dirname(os.path.abspath(__file__))
OUT_ROOT = os.path.join(ROOT, "experiments", "data", "attestation-sc")
SC = os.path.join(ROOT, "smart-contracts")
COLUMNS = ["agents", "eligible_before", "scenario", "function", "path", "gas", "calldata_bytes", "log_bytes"]

# (scenario, function, path) every number of agents must have.
EXPECTED = [
    ("setup", "RegisterAgent", "register"),
    ("setup", "SetReference", "approval"),
    ("setup", "SetReference", "final_approval"),
    ("bootstrap", "Prove", "to_secaas"),
    ("bootstrap", "VerifyBySECaaS", "bootstrap"),
    ("steady", "Prove", "to_jury"),
    ("steady", "Vote", "first"),
    ("steady", "Vote", "closing"),
    ("failure", "Vote", "first_against"),
    ("failure", "Vote", "closing_against"),
    ("dissent", "Vote", "to_secaas_against"),
    ("dissent", "VerifyBySECaaS", "after_dissent"),
    ("expiry", "Expire", "expire"),
    ("expiry", "VerifyBySECaaS", "after_expiry"),
]


def read_env():
    env = {}
    path = os.path.join(ROOT, ".env")
    if os.path.isfile(path):
        with open(path) as f:
            for line in f:
                key, sep, val = line.strip().partition("=")
                if sep and not key.startswith("#"):
                    env[key.strip()] = val.strip()
    return env


def next_run_dir(base):
    os.makedirs(base, exist_ok=True)
    taken = [int(d[3:]) for d in os.listdir(base) if d.startswith("run") and d[3:].isdigit()]
    return os.path.join(base, f"run{max(taken, default=0) + 1:02d}")


def main():
    env = read_env()
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--agents", default="4,8,16,32,64,128", help="numbers of agents (default: 4,8,16,32,64,128)")
    p.add_argument("--rounds", type=int, default=20, help="normal rounds per number of agents (default: 20)")
    p.add_argument("--failures", type=int, default=5, help="rounds per failure, dissent, expiry path (default: 5)")
    p.add_argument("--eligible", default="1,0.5,0.25", help="fractions of eligible agents (default: 1,0.5,0.25)")
    p.add_argument("--fraction-agents", type=int, default=64, help="agents of the eligibility sweep (default: 64)")
    p.add_argument("--jury", type=int, default=int(env.get("JURY_SIZE") or 2))
    p.add_argument("--threshold", type=int, default=int(env.get("JURY_THRESHOLD") or 2))
    p.add_argument("--timeout", type=int, default=int(env.get("JURY_TIMEOUT_BLOCKS") or 5))
    p.add_argument("--freshness", type=int, default=int(env.get("FRESHNESS_BLOCKS") or 15))
    args = p.parse_args()
    agents = [int(a) for a in args.agents.split(",")]

    params = {"AGENTS": args.agents, "ROUNDS": args.rounds, "FAILURES": args.failures, "ELIGIBLE": args.eligible,
              "FRACTION_AGENTS": args.fraction_agents, "JURY_SIZE": args.jury, "JURY_THRESHOLD": args.threshold,
              "JURY_TIMEOUT_BLOCKS": args.timeout, "FRESHNESS_BLOCKS": args.freshness}
    print(f"jury {args.threshold} of {args.jury}, agents {args.agents}, {args.rounds} rounds each")
    t0 = time.time()
    cmd = ["docker", "run", "--rm",
           "-v", f"{SC}/scripts:/smart-contracts/scripts",
           "-v", f"{SC}/contracts:/smart-contracts/contracts",
           "-v", f"{SC}/artifacts:/smart-contracts/artifacts",
           "-v", f"{SC}/hardhat.config.js:/smart-contracts/hardhat.config.js",
           "-v", "hardhat-compilers:/root/.cache/hardhat-nodejs"]
    for k, v in params.items():
        cmd += ["-e", f"{k}={v}"]
    cmd += ["hardhat:latest", "bash", "-lc", "npx hardhat run scripts/characterize_gas.js"]
    out = subprocess.run(cmd, capture_output=True, text=True)
    lines = out.stdout.splitlines()
    if out.returncode != 0 or "DONE" not in lines:
        sys.exit(f"the characterization failed:\n{out.stdout[-3000:]}{out.stderr[-3000:]}")
    info = next(json.loads(l[5:]) for l in lines if l.startswith("INFO "))
    rows = [json.loads(l[4:]) for l in lines if l.startswith("GAS ")]

    problems = []
    seen = {(r["agents"], r["scenario"], r["function"], r["path"]) for r in rows}
    for n in agents:
        missing = [e for e in EXPECTED if (n,) + e not in seen]
        if missing:
            problems.append(f"{n} agents: no {missing}")
    sweep = {r["eligible_before"] for r in rows if r["scenario"] == "eligibility"}
    if len(sweep) < len(args.eligible.split(",")):
        problems.append(f"eligibility sweep: eligible counts {sorted(sweep)}")
    print(f"{len(rows)} transactions in {time.time() - t0:.0f} s, {len(problems)} problem(s)")
    for problem in problems:
        print(f"⚠️  {problem}")

    with open(os.path.join(SC, "contracts", "AttestationManagerV3.sol"), "rb") as f:
        source_sha = hashlib.sha256(f.read()).hexdigest()
    meta = {
        **info, "contract": "AttestationManagerV3", "contract_sha256": source_sha,
        "solidity": "0.8.28, optimizer 200 runs (smart-contracts/hardhat.config.js)",
        "transactions": len(rows), "problems": problems,
        "t_start": time.strftime("%Y-%m-%dT%H:%M:%S%z", time.localtime(t0)),
        "git_commit": subprocess.run(["git", "-C", ROOT, "rev-parse", "--short", "HEAD"],
                                     capture_output=True, text=True).stdout.strip(),
    }
    path = next_run_dir(os.path.join(OUT_ROOT, f"J{args.jury}-T{args.threshold}"))
    tmp = path + ".tmp"
    os.makedirs(tmp)
    with open(os.path.join(tmp, "gas.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    with open(os.path.join(tmp, "meta.json"), "w") as f:
        json.dump(meta, f, indent=2)
    os.rename(tmp, path)
    print(f"→ {os.path.relpath(path, ROOT)}")
    sys.exit(1 if problems else 0)


if __name__ == "__main__":
    main()
