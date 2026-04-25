#!/usr/bin/env python3
"""
Performance benchmark orchestrator.

Measures ROS2 /scan topic jitter with and without attestation sidecars
to verify the sidecar introduces no robot performance penalty.

Usage:
  python3 run_benchmark.py --condition no_sidecar  --mode continuous --runs 5
  python3 run_benchmark.py --condition with_sidecar --mode continuous --runs 5
  python3 run_benchmark.py --condition no_sidecar  --mode startup    --runs 5
  python3 run_benchmark.py --condition with_sidecar --mode startup    --runs 5

Prerequisites:
  docker-compose -f docker-compose.robots.yml \\
                 -f docker-compose.benchmark-collector.yml up -d
  (add docker-compose.attestation.yml for with_sidecar conditions)
"""

import argparse
import os
import sys
import time
from concurrent.futures import ThreadPoolExecutor, as_completed
from pathlib import Path

import requests

COLLECTOR_URL    = "http://localhost:7001"
SECAAS_URL       = "http://localhost:8000"
SECAAS_PORT      = 8000
ROBOTS_BASE_PORT = 8001
N_ROBOTS         = 4
DURATION_S       = 120

BASE_RESULTS_DIR      = Path(__file__).parent / "experiments/data/performance-benchmark/results"
COLLECTOR_RESULTS_DIR = "/experiments/data/performance-benchmark/results"   # container-side path


# ── Sidecar helpers (mirrors run_experiments_and_collect_results.py) ───────────

def build_sidecars():
    s = {"secaas": SECAAS_PORT}
    for i in range(1, N_ROBOTS + 1):
        s[f"robot{i}"] = ROBOTS_BASE_PORT + i - 1
    return s


def wait_for_http(url, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        try:
            requests.get(url, timeout=2)
            return True
        except requests.RequestException:
            time.sleep(1)
    return False


def trigger_action(name, port, action, timeout=5):
    try:
        resp = requests.post(f"http://localhost:{port}/{action}", timeout=timeout)
        resp.raise_for_status()
        return f"✔ {name}: {action}"
    except Exception as e:
        return f"❌ {name}: {e}"


def get_status(name, port):
    try:
        return requests.get(f"http://localhost:{port}/status", timeout=5).json().get("status", "unknown")
    except Exception:
        return "error"


def _poll_parallel(robot_sidecars):
    with ThreadPoolExecutor(max_workers=len(robot_sidecars)) as ex:
        futs = {ex.submit(get_status, n, p): n for n, p in robot_sidecars.items()}
        return {futs[f]: f.result() for f in as_completed(futs)}


def wait_for_all_running(robot_sidecars, timeout=30):
    start = time.time()
    while time.time() - start < timeout:
        if all(s in ("running", "finished") for s in _poll_parallel(robot_sidecars).values()):
            return True
        time.sleep(0.5)
    return False


def wait_for_all_finished(robot_sidecars, timeout=120):
    start = time.time()
    while time.time() - start < timeout:
        if all(s == "finished" for s in _poll_parallel(robot_sidecars).values()):
            return True
        time.sleep(2)
    return False


def set_config(name, port, payload):
    try:
        requests.post(f"http://localhost:{port}/config", json=payload, timeout=5).raise_for_status()
    except Exception:
        pass


def set_config_all(robot_sidecars, payload):
    with ThreadPoolExecutor(max_workers=len(robot_sidecars)) as ex:
        futs = [ex.submit(set_config, n, p, payload) for n, p in robot_sidecars.items()]
        for f in as_completed(futs):
            pass


def stop_all(sidecars):
    with ThreadPoolExecutor(max_workers=len(sidecars)) as ex:
        futs = [ex.submit(trigger_action, n, p, "stop", 15) for n, p in sidecars.items()]
        for f in as_completed(futs):
            print(f"      {f.result()}")


def reset_chain():
    try:
        requests.post(f"{SECAAS_URL}/reset", timeout=10)
        print("   [Chain] Reset.")
    except Exception as e:
        print(f"   [Chain] ⚠️ {e}")


def run_warmup(sidecars, robot_sidecars):
    """One-shot warmup with export disabled (seeds lastSuccess on chain)."""
    print("🔥 Warmup: seeding lastSuccess...")
    set_config_all(robot_sidecars, {"one_shot": True, "export_enabled": False})
    set_config("secaas", SECAAS_PORT, {"export_enabled": False})
    with ThreadPoolExecutor(max_workers=len(sidecars)) as ex:
        for f in as_completed([ex.submit(trigger_action, n, p, "start") for n, p in sidecars.items()]):
            pass
    wait_for_all_running(robot_sidecars)
    wait_for_all_finished(robot_sidecars)
    stop_all(sidecars)
    print("✅ Warmup done.\n")


# ── Collector helpers ──────────────────────────────────────────────────────────

def wait_for_topics_active(min_msgs=N_ROBOTS, timeout=60):
    """Block until collector reports at least min_msgs received (one per robot)."""
    print(f"   ⏳ Waiting for /scan topics ({min_msgs} msgs)...")
    start = time.time()
    while time.time() - start < timeout:
        try:
            n = requests.get(f"{COLLECTOR_URL}/status", timeout=3).json().get("messages_received", 0)
            if n >= min_msgs:
                print(f"   ✅ Topics active ({n} msgs received).")
                return True
        except Exception:
            pass
        time.sleep(1)
    print("   ⚠️ Timeout waiting for topics.")
    return False


# ── Run loop ───────────────────────────────────────────────────────────────────

def run_benchmark_loop(run_id, condition, mode, topic_slug, sidecars, robot_sidecars):
    print(f"\n{'='*55}\n🏁 Run {run_id}  [{condition} / {mode} / {topic_slug}]\n{'='*55}")

    host_output      = BASE_RESULTS_DIR / topic_slug / condition / mode / f"run{run_id}.csv"
    container_output = f"{COLLECTOR_RESULTS_DIR}/{topic_slug}/{condition}/{mode}/run{run_id}.csv"
    host_output.parent.mkdir(parents=True, exist_ok=True)

    # Reset collector state from previous run
    requests.post(f"{COLLECTOR_URL}/reset", timeout=5)

    # Wait until robots are publishing before starting the measurement window
    wait_for_topics_active(min_msgs=N_ROBOTS, timeout=60)

    if condition == "with_sidecar":
        set_config_all(robot_sidecars, {
            "one_shot":       mode == "startup",
            "export_enabled": False,
        })
        set_config("secaas", SECAAS_PORT, {"export_enabled": False})

        print("   [Agents] Starting...")
        with ThreadPoolExecutor(max_workers=len(sidecars)) as ex:
            for f in as_completed([ex.submit(trigger_action, n, p, "start") for n, p in sidecars.items()]):
                print(f"      {f.result()}")
        wait_for_all_running(robot_sidecars)

    # Start collector only after sidecars are up (or immediately for baseline)
    print(f"   [Collector] Recording → {host_output}")
    requests.post(f"{COLLECTOR_URL}/start", json={
        "output_file": container_output,
        "duration_s":  DURATION_S + 10,    # buffer; we stop explicitly below
    }, timeout=5)
    t_collect_start = time.time()

    if condition == "with_sidecar":
        if mode == "startup":
            wait_for_all_finished(robot_sidecars, timeout=120)
            stop_all(sidecars)
            # Pad so the collection window always covers DURATION_S
            remaining = DURATION_S - (time.time() - t_collect_start)
            if remaining > 0:
                print(f"   ⏳ Collector padding {remaining:.1f}s...")
                time.sleep(remaining)
            reset_chain()
        else:
            print(f"   ⏳ Running for {DURATION_S}s...")
            time.sleep(DURATION_S)
            stop_all(sidecars)
    else:
        print(f"   ⏳ Baseline for {DURATION_S}s...")
        time.sleep(DURATION_S)

    # Stop collector
    resp = requests.post(f"{COLLECTOR_URL}/stop", timeout=10)
    print(f"   [Collector] {resp.json().get('records', '?')} records saved.")


# ── Main ───────────────────────────────────────────────────────────────────────

def main():
    parser = argparse.ArgumentParser(
        description="ROS2 topic jitter benchmark — with vs without attestation sidecar",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--condition", choices=["no_sidecar", "with_sidecar"], required=True)
    parser.add_argument("--mode",      choices=["startup", "continuous"],
                        help="Required for with_sidecar; ignored for no_sidecar (always 'baseline').")
    parser.add_argument("--runs",      type=int, default=5)
    parser.add_argument("--topic",     default="scan",
                        help="Topic suffix used for this run, e.g. 'scan' or 'odom'. "
                             "Must match TOPIC_NAME set when starting the collector. "
                             "Used only to organise output paths. (default: scan)")
    args = parser.parse_args()

    # no_sidecar has no attestation cycle — startup vs continuous is meaningless
    if args.condition == "no_sidecar":
        mode = "baseline"
    else:
        if args.mode is None:
            parser.error("--mode is required for --condition with_sidecar")
        mode = args.mode

    topic_slug = args.topic.lstrip("/").replace("/", "_") or "scan"

    sidecars       = build_sidecars()
    robot_sidecars = {k: v for k, v in sidecars.items() if k != "secaas"}

    print(f"📋 condition={args.condition}  mode={mode}  topic={topic_slug}  runs={args.runs}")
    print(f"📁 Output: {BASE_RESULTS_DIR / topic_slug / args.condition / mode}/")

    # Pre-flight: collector
    print("\n🔍 Checking topic collector...")
    if not wait_for_http(f"{COLLECTOR_URL}/status", timeout=30):
        print(f"❌ Collector not reachable at {COLLECTOR_URL}")
        sys.exit(1)
    print("✅ Collector ready.")

    # Pre-flight: sidecars (only for with_sidecar)
    if args.condition == "with_sidecar":
        print("🔍 Checking sidecars...")
        for name, port in sidecars.items():
            if not wait_for_http(f"http://localhost:{port}/", timeout=15):
                print(f"❌ Sidecar '{name}' not reachable.")
                sys.exit(1)
        print("✅ Sidecars ready.")

        if mode == "continuous":
            run_warmup(sidecars, robot_sidecars)
            set_config_all(robot_sidecars, {"export_enabled": False})
            set_config("secaas", SECAAS_PORT, {"export_enabled": False})
            print("   [System] Cooldown 5s after warmup...")
            time.sleep(5)

    for i in range(1, args.runs + 1):
        run_benchmark_loop(i, args.condition, mode, topic_slug, sidecars, robot_sidecars)
        if i < args.runs:
            print("   [System] Cooldown 5s...")
            time.sleep(5)

    print(f"\n🎉 Done. Results in {BASE_RESULTS_DIR / topic_slug / args.condition / mode}/")


if __name__ == "__main__":
    main()
