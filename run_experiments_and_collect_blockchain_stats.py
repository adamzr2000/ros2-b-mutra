#!/usr/bin/env python3
"""Start attestation agents and collect blockchain stats for a single fixed-duration run.

Run index is determined automatically: the script scans the output directory and picks
the next available run number, so repeated invocations never overwrite existing results.

Examples:
  # Local (all containers on this machine)
  python3 run_experiments_and_collect_blockchain_stats.py --robots 8 --ssp 20000 --iterq 1 --duration 300

  # Remote robots
  python3 run_experiments_and_collect_blockchain_stats.py --robots 8 --ssp 20000 --iterq 1 --duration 300 --remote
"""

import argparse
import json
import re
import sys
import time

import requests
from concurrent.futures import ThreadPoolExecutor, as_completed

# ── Constants ──────────────────────────────────────────────────────────────────
ETH_MONITOR_URL  = "http://localhost:7000"
ETH_RPC_URL      = "http://host.docker.internal:21001"
SECAAS_PORT      = 8000
ROBOTS_BASE_PORT = 8001

_DEFAULT_REMOTE1_HOST = "10.5.1.20"
REMOTE_USER           = "nextnet"

_BASE_RESULTS = "/experiments/data/blockchain-stats/results"

_SECAAS_HP = ("localhost", SECAAS_PORT)


# ── Run-number helpers ─────────────────────────────────────────────────────────

def _next_run(csv_dir: str, ssp: int, iterq: int) -> int:
    """Return the next available run index by scanning existing files in csv_dir."""
    import os
    pattern = re.compile(
        rf"blockchain-SSP{ssp}ms-ITERQ{iterq}-run(\d+)\.csv$", re.IGNORECASE
    )
    max_run = 0
    try:
        for name in os.listdir(csv_dir):
            m = pattern.match(name)
            if m:
                max_run = max(max_run, int(m.group(1)))
    except FileNotFoundError:
        pass
    return max_run + 1


# ── Sidecar helpers ────────────────────────────────────────────────────────────

def build_sidecars(num_robots, remote1_host=None):
    sidecars = {"secaas": _SECAAS_HP}
    for i in range(1, num_robots + 1):
        host = remote1_host if remote1_host else "localhost"
        sidecars[f"robot{i}"] = (host, ROBOTS_BASE_PORT + i - 1)
    return sidecars


def _trigger(name, host_port, action, timeout=15):
    host, port = host_port
    try:
        requests.post(f"http://{host}:{port}/{action}", timeout=timeout).raise_for_status()
        return f"✔ {name}: {action.upper()}"
    except Exception as e:
        return f"❌ {name}: {e}"


def _set_config(name, host_port, payload):
    host, port = host_port
    try:
        resp = requests.post(f"http://{host}:{port}/config", json=payload, timeout=5)
        resp.raise_for_status()
        return f"✔ {name}: {resp.json().get('applied', payload)}"
    except Exception as e:
        return f"❌ {name}: {e}"


def _parallel(fn, targets):
    with ThreadPoolExecutor(max_workers=len(targets)) as ex:
        futures = [ex.submit(fn, *t) for t in targets]
        for f in as_completed(futures):
            print(f"   {f.result()}")


# ── Eth monitor helpers ────────────────────────────────────────────────────────

def _post(url, payload=None, timeout=10):
    resp = requests.post(url, json=payload, timeout=timeout)
    resp.raise_for_status()
    try:
        return resp.json()
    except ValueError:
        return {"raw": resp.text}


def _get(url, timeout=5):
    resp = requests.get(url, timeout=timeout)
    resp.raise_for_status()
    try:
        return resp.json()
    except ValueError:
        return {"raw": resp.text}


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="Run attestation agents and collect blockchain stats for one fixed-duration run.",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--robots",   type=int, required=True,
                        help="Number of robot sidecars")
    parser.add_argument("--duration", type=int, required=True,
                        help="Run duration in seconds")
    parser.add_argument("--ssp",      type=int, required=True,
                        help="Attestation interval in ms (for filename labelling, e.g. 20000)")
    parser.add_argument("--iterq",    type=int, default=1,
                        help="Rolling-hash queue depth (for filename labelling, default: 1)")
    parser.add_argument("--remote",   action="store_true",
                        help="Robot sidecars are on a remote host instead of localhost")
    parser.add_argument("--remote1-host", default=None,
                        help=f"Remote host IP (default: {_DEFAULT_REMOTE1_HOST})")
    parser.add_argument("--poll-interval", type=float, default=1.0,
                        help="Blockchain monitor poll interval in seconds (default: 1.0)")
    args = parser.parse_args()

    remote1_host = (args.remote1_host or _DEFAULT_REMOTE1_HOST) if args.remote else None
    sidecars     = build_sidecars(args.robots, remote1_host)
    robot_sidecars = {k: v for k, v in sidecars.items() if k != "secaas"}

    csv_dir  = f"{_BASE_RESULTS}/N{args.robots}"
    run      = _next_run(csv_dir, args.ssp, args.iterq)
    csv_name = f"blockchain-SSP{args.ssp}ms-ITERQ{args.iterq}-run{run}.csv"

    print(f"📋 Robots: {args.robots}  SSP: {args.ssp}ms  ITERQ: {args.iterq}  run: {run}")
    print(f"⏱  Duration: {args.duration}s")
    print(f"📁 Output: {csv_dir}/{csv_name}")
    if remote1_host:
        print(f"🌐 Remote robots on: {remote1_host}")

    # 1. Health check
    print(f"\n[1/5] Health check -> {ETH_MONITOR_URL}")
    try:
        print(json.dumps(_get(f"{ETH_MONITOR_URL}/", timeout=5), indent=2))
    except Exception as e:
        print(f"❌ Eth monitor unreachable: {e}")
        sys.exit(1)

    # 2. Configure agents — push SSP/ITERQ and disable export (we only care about chain stats)
    print("\n[2/5] Configuring agents (SSP/ITERQ/export)...")
    robot_cfg = {
        "one_shot":               False,
        "export_enabled":         False,
        "attestation_interval_ms": args.ssp,
        "iterq_threshold":        args.iterq,
    }
    _parallel(_set_config, [
        (name, hp, robot_cfg)
        for name, hp in robot_sidecars.items()
    ])
    print(f"   {_set_config('secaas', _SECAAS_HP, {'export_enabled': False})}")

    # 3. Start blockchain monitor
    print("\n[3/5] Starting blockchain monitor...")
    payload = {
        "rpc_url":       ETH_RPC_URL,
        "poll_interval": args.poll_interval,
        "csv_dir":       csv_dir,
        "csv_name":      csv_name,
    }
    try:
        print(json.dumps(_post(f"{ETH_MONITOR_URL}/monitor/start", payload), indent=2))
    except Exception as e:
        print(f"❌ Failed to start monitor: {e}")
        sys.exit(1)

    # 4. Start agents
    print("\n[4/5] Starting agents...")
    _parallel(_trigger, [(name, hp, "start") for name, hp in sidecars.items()])

    # 5. Run for duration, then stop everything
    print(f"\n[5/5] Running for {args.duration}s...")
    try:
        time.sleep(args.duration)
    except KeyboardInterrupt:
        print("\n⚠️  Interrupted — stopping agents and monitor...")

    print("   Stopping agents...")
    _parallel(_trigger, [(name, hp, "stop") for name, hp in sidecars.items()])

    print("   Stopping blockchain monitor...")
    try:
        _post(f"{ETH_MONITOR_URL}/monitor/stop", timeout=20)
    except Exception as e:
        print(f"⚠️  Monitor stop failed: {e}")

    print(f"\n✅ Done. Results saved to {csv_dir}/{csv_name}")


if __name__ == "__main__":
    main()
