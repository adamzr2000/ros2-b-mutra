#!/usr/bin/env python3
import os
import time
import argparse
import subprocess
import json
import requests
from concurrent.futures import ThreadPoolExecutor, as_completed

# --- Configuration Defaults ---
DEFAULTS = {
    "results_dir": "/experiments/data/attestation-times/results",
    "stats_dir": "/experiments/data/docker-stats/results",
    "duration": 60,
    "runs": 1,
    "password": "netcom;", # Default sudo password
    "env_file": ".env"
}

# Map names to ports
SERVICE_PORTS = {
    "secaas": 8080,
    "robot1": 8081,
    "robot2": 8082,
    "robot3": 8083,
    "robot4": 8084
}

MONITOR_URL = "http://localhost:6000"

# --- Helper Functions ---

def run_command(cmd, shell=False, check=True, input_str=None):
    """Run a shell command."""
    try:
        # If input_str is provided (for sudo password), encode it
        input_bytes = input_str.encode() if input_str else None
        subprocess.run(cmd, shell=shell, check=check, input=input_bytes, stderr=subprocess.DEVNULL, stdout=subprocess.DEVNULL)
    except subprocess.CalledProcessError as e:
        # We silence output for cleanliness, but print on error
        print(f"❌ Command failed: {cmd}")
        raise e

def upsert_env(key, value):
    """Safely updates or adds a key=value pair in the .env file."""
    env_path = os.path.join(os.getcwd(), DEFAULTS["env_file"])
    
    # Ensure file exists
    if not os.path.exists(env_path):
        open(env_path, 'w').close()

    with open(env_path, 'r') as f:
        lines = f.readlines()

    key_found = False
    new_lines = []
    
    for line in lines:
        if line.strip().startswith(f"{key}="):
            new_lines.append(f"{key}={value}\n")
            key_found = True
        else:
            new_lines.append(line)
            
    if not key_found:
        # Ensure we start on a new line if the file doesn't end with one
        if new_lines and not new_lines[-1].endswith('\n'):
            new_lines[-1] += '\n'
        new_lines.append(f"{key}={value}\n")

    with open(env_path, 'w') as f:
        f.writelines(new_lines)

def wait_for_http(url, timeout=30):
    """Waits for an HTTP endpoint to become available."""
    start_time = time.time()
    while time.time() - start_time < timeout:
        try:
            requests.get(url, timeout=2)
            return True
        except requests.RequestException:
            time.sleep(0.5)
    print(f"⚠️  Timeout waiting for {url}")
    return False

def trigger_request(name, port, endpoint):
    """Sends a POST request to a specific sidecar (for parallel execution)."""
    url = f"http://localhost:{port}/{endpoint}"
    try:
        resp = requests.post(url, timeout=5)
        resp.raise_for_status()
        return f"✔ {name}: {json.dumps(resp.json())}"
    except Exception as e:
        return f"❌ {name} failed: {e}"

def reset_chain():
    """Resets the blockchain state."""
    print("🔁 Resetting chain on SECaaS...")
    try:
        resp = requests.post("http://localhost:8080/reset", timeout=20)
        print(json.dumps(resp.json(), indent=2))
    except Exception as e:
        print(f"⚠️  Chain reset failed: {e}")

def cleanup(password):
    """Stops monitors and tears down docker compose."""
    print("\n🧹 Cleaning up...")
    try:
        requests.post(f"{MONITOR_URL}/monitor/stop", timeout=2)
    except:
        pass
    
    # Run docker compose down with sudo
    cmd = f"sudo -S docker compose down"
    # We pipe the password to stdin
    try:
        run_command(cmd, shell=True, input_str=f"{password}\n", check=False)
    except Exception as e:
        print(f"Error during cleanup: {e}")

# --- Main Logic ---

def main():
    parser = argparse.ArgumentParser(description="Run full experiments.")
    parser.add_argument("--results-dir", default=DEFAULTS["results_dir"])
    parser.add_argument("--stats-dir", default=DEFAULTS["stats_dir"])
    parser.add_argument("--duration", type=int, default=DEFAULTS["duration"])
    parser.add_argument("--runs", type=int, default=DEFAULTS["runs"])
    args = parser.parse_args()

    # Password setup (ENV var takes precedence over default)
    sudo_pass = os.getenv("PASSWORD", DEFAULTS["password"])

    # 1. Setup .env
    upsert_env("EXPORT_RESULTS", "TRUE")
    upsert_env("RESULTS_DIR", args.results_dir)
    upsert_env("AUTO_START", "FALSE")
    print(f"✅ .env updated: RESULTS_DIR={args.results_dir}")

    try:
        for run in range(1, args.runs + 1):
            print(f"\n==============================\n🏁 Run {run} / {args.runs}\n==============================")
            upsert_env("RUN_ID", run)

            # 2. Start Containers
            print("[+] Starting containers...")
            run_command("docker compose up -d", shell=True)
            time.sleep(5)

            # 3. Wait for Services
            print("[+] Waiting for readiness...")
            wait_for_http(f"{MONITOR_URL}/monitor/status")
            for name, port in SERVICE_PORTS.items():
                wait_for_http(f"http://localhost:{port}/")

            # 4. Start Docker Stats
            print(f"📊 Starting docker-stats → {args.stats_dir}")
            containers = ["secaas", "robot1-sidecar", "robot2-sidecar", "robot3-sidecar", "robot4-sidecar"]
            payload = {
                "containers": containers,
                "interval": 1.0,
                "csv_dir": args.stats_dir,
                "stdout": True
            }
            try:
                r = requests.post(f"{MONITOR_URL}/monitor/start", json=payload)
                print(json.dumps(r.json(), indent=2))
            except Exception as e:
                print(f"❌ Failed to start monitor: {e}")

            # 5. Start Attestations (PARALLEL)
            print("▶️  Starting attestations (Parallel)...")
            with ThreadPoolExecutor(max_workers=len(SERVICE_PORTS)) as executor:
                futures = {executor.submit(trigger_request, name, port, "start"): name for name, port in SERVICE_PORTS.items()}
                for future in as_completed(futures):
                    print(future.result())

            # 6. Run Experiment
            print(f"⏱  Running for {args.duration}s...")
            time.sleep(args.duration)

            # 7. Stop Attestations (PARALLEL)
            print("⏹  Stopping attestations (Parallel)...")
            with ThreadPoolExecutor(max_workers=len(SERVICE_PORTS)) as executor:
                futures = {executor.submit(trigger_request, name, port, "stop"): name for name, port in SERVICE_PORTS.items()}
                for future in as_completed(futures):
                    print(future.result())

            # 8. Stop Docker Stats
            print("⏹  Stopping docker-stats...")
            try:
                r = requests.post(f"{MONITOR_URL}/monitor/stop")
                print(json.dumps(r.json(), indent=2))
            except:
                pass

            # 9. Reset Chain
            reset_chain()

            # 10. Teardown
            print("🧹  docker compose down...")
            run_command("sudo -S docker compose down", shell=True, input_str=f"{sudo_pass}\n")

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        # Remove RUN_ID from .env is a bit complex, but keeping it is harmless.
        # If you really want to remove it, you'd implement a remove_env_key function.
        cleanup(sudo_pass)
        print(f"🎉 Done. Results in:\n  Attestations: {args.results_dir}\n  Docker stats: {args.stats_dir}")

if __name__ == "__main__":
    main()