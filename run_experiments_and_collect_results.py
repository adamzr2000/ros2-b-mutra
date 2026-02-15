#!/usr/bin/env python3
import time
import argparse
import requests
import sys
import os  # <--- Added this
from concurrent.futures import ThreadPoolExecutor, as_completed

# --- Configuration ---
MONITOR_URL = "http://localhost:6000"
SECAAS_URL = "http://localhost:8000"

# Map of service names to their sidecar ports
SIDE_CARS = {
    "secaas": 8000,
    "robot1": 8001,
    "robot2": 8002,
    "robot3": 8003,
    "robot4": 8004,
}

# --- Helper Functions ---

def wait_for_http(url, timeout=30):
    """Waits for an HTTP endpoint to become available."""
    print(f"⏳ Waiting for {url} ...", end="", flush=True)
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            requests.get(url, timeout=2)
            print(" ✅")
            return True
        except requests.RequestException:
            time.sleep(1)
            print(".", end="", flush=True)
    print("\n❌ Timeout waiting for service.")
    return False

def check_all_services():
    """Verifies that ALL defined services are up and listening."""
    print("\n🔍 Pre-flight check: Verifying all services...")

    # 1. Check Monitor
    if not wait_for_http(f"{MONITOR_URL}/monitor/status"):
        return False

    # 2. Check SECaaS and Robots
    for name, port in SIDE_CARS.items():
        url = f"http://localhost:{port}/"
        if not wait_for_http(url):
            print(f"❌ Service '{name}' on port {port} is not responding.")
            return False

    print("🚀 All systems GO.\n")
    return True

def trigger_action(name, port, action):
    """Sends start/stop signals to the sidecar API."""
    try:
        url = f"http://localhost:{port}/{action}"
        resp = requests.post(url, timeout=5)
        resp.raise_for_status()
        return f"✔ {name}: {action.upper()}"
    except Exception as e:
        return f"❌ {name}: Failed to {action} ({e})"

def reset_chain():
    """Resets the blockchain state via SECaaS API."""
    try:
        requests.post(f"{SECAAS_URL}/reset", timeout=10)
        print("   [Chain] State reset successfully.")
    except Exception as e:
        print(f"   [Chain] ⚠️ Warning: Reset failed: {e}")

def run_experiment_loop(run_id, duration, stats_dir):
    print(f"\n==============================\n🏁 Run {run_id}\n==============================")

    # 1. Start Docker Monitor
    print(f"   [Monitor] Starting stats -> {stats_dir}")
    try:
        requests.post(f"{MONITOR_URL}/monitor/start", json={
            "containers": ["secaas"] + [f"{r}-sidecar" for r in SIDE_CARS if r != "secaas"],
            "interval": 1.0,
            "csv_dir": stats_dir,
            "run_id": run_id
        }, timeout=5)
    except Exception as e:
        print(f"   ⚠️ Monitor start failed: {e}")

    # 2. Start Agents (Parallel)
    print("   [Agents] Starting...")
    with ThreadPoolExecutor(max_workers=len(SIDE_CARS)) as executor:
        futures = [executor.submit(trigger_action, n, p, "start") for n, p in SIDE_CARS.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    # 3. Wait
    print(f"   ⏳ Running for {duration}s...")
    time.sleep(duration)

    # 4. Stop Agents (Parallel)
    print("   [Agents] Stopping...")
    with ThreadPoolExecutor(max_workers=len(SIDE_CARS)) as executor:
        futures = [executor.submit(trigger_action, n, p, "stop") for n, p in SIDE_CARS.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    # 5. Stop Monitor & Reset
    try:
        requests.post(f"{MONITOR_URL}/monitor/stop", timeout=5)
    except Exception:
        pass

    reset_chain()

# --- Main ---

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Data collection")
    parser.add_argument("--runs", type=int, default=1, help="Number of runs")
    parser.add_argument("--duration", type=int, default=60, help="Duration in seconds")
    parser.add_argument("--stats-dir", default="/experiments/data/docker-stats/results", help="Output directory")

    args = parser.parse_args()

    # 1. Wait for ALL services to be ready
    if not check_all_services():
        print("🛑 Aborting: Not all services are ready.")
        sys.exit(1)

    # 2. Run the experiment loop
    try:
        for i in range(1, args.runs + 1):
            run_experiment_loop(i, args.duration, args.stats_dir)
            if i < args.runs:
                print("   [System] Cooling down for 5s...")
                time.sleep(5)

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")

    print(f"\n🎉 All runs completed. Results saved to {args.stats_dir}")

    # --- Cleanup Ghost Artifacts ---
    ghost_file = "experiments/data/attestation-times/results/secaas.json"
    if os.path.exists(ghost_file):
        try:
            os.remove(ghost_file)
            print(f"🧹 Cleaned up artifact: {ghost_file}")
        except OSError:
            pass