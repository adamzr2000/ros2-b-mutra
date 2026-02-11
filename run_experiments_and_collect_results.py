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
    "password": "netcom;",  # Default sudo password
    "env_file": ".env",
}

# Compose config (assumes ./start.sh already brought up chain + robots + secaas + monitoring)
PROJECT_NAME = os.getenv("COMPOSE_PROJECT_NAME", "ros2-b-mutra")
ATTESTATION_COMPOSE_FILE = "docker-compose.attestation.yml"

# Map names to ports
SERVICE_PORTS = {
    "secaas": 8000,
    "robot1": 8001,
    "robot2": 8002,
    "robot3": 8003,
    "robot4": 8004,
}

MONITOR_URL = "http://localhost:6000"

# --- Helper Functions ---


def run_command(cmd, shell=False, check=True, input_str=None, env=None):
    """Run a shell command."""
    try:
        input_bytes = input_str.encode() if input_str else None
        subprocess.run(
            cmd,
            shell=shell,
            check=check,
            input=input_bytes,
            stderr=subprocess.DEVNULL,
            stdout=subprocess.DEVNULL,
            env=env,
        )
    except subprocess.CalledProcessError as e:
        print(f"❌ Command failed: {cmd}")
        raise e


def upsert_env(key, value):
    """Safely updates or adds a key=value pair in the .env file."""
    env_path = os.path.join(os.getcwd(), DEFAULTS["env_file"])

    if not os.path.exists(env_path):
        open(env_path, "w").close()

    with open(env_path, "r") as f:
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
        if new_lines and not new_lines[-1].endswith("\n"):
            new_lines[-1] += "\n"
        new_lines.append(f"{key}={value}\n")

    with open(env_path, "w") as f:
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
        resp = requests.post("http://localhost:8000/reset", timeout=20)
        print(json.dumps(resp.json(), indent=2))
    except Exception as e:
        print(f"⚠️  Chain reset failed: {e}")


def compose_up_attestation(compose_env):
    run_command(
        f"docker compose -p {PROJECT_NAME} -f {ATTESTATION_COMPOSE_FILE} up -d",
        shell=True,
        env=compose_env,
    )


def compose_down_attestation(password, compose_env):
    run_command(
        f"sudo -S docker compose -p {PROJECT_NAME} -f {ATTESTATION_COMPOSE_FILE} down",
        shell=True,
        input_str=f"{password}\n",
        check=False,
        env=compose_env,
    )


def restart_secaas(password, timeout=60):
    """
    Restart SECaaS so it re-reads RUN_ID on startup and selects secaas-run{RUN_ID}.json.
    (RUN_ID is only processed in SECaaS startup code.)
    """
    print("🔁 Restarting SECaaS (to apply RUN_ID)...")
    run_command(
        "sudo -S docker restart secaas",
        shell=True,
        input_str=f"{password}\n",
        check=True,
    )
    if not wait_for_http("http://localhost:8000/", timeout=timeout):
        raise RuntimeError("SECaaS did not become ready after restart.")


def cleanup(password, compose_env):
    """Stops monitors and tears down attestation stack."""
    print("\n🧹 Cleaning up...")
    try:
        requests.post(f"{MONITOR_URL}/monitor/stop", timeout=2)
    except Exception:
        pass

    try:
        compose_down_attestation(password, compose_env)
    except Exception as e:
        print(f"Error during cleanup: {e}")


# --- Main Logic ---


def main():
    parser = argparse.ArgumentParser(
        description="Run experiments (assumes ./start.sh already ran with --no-auto --no-export)."
    )
    parser.add_argument("--results-dir", default=DEFAULTS["results_dir"])
    parser.add_argument("--stats-dir", default=DEFAULTS["stats_dir"])
    parser.add_argument("--duration", type=int, default=DEFAULTS["duration"])
    parser.add_argument("--runs", type=int, default=DEFAULTS["runs"])
    args = parser.parse_args()

    sudo_pass = os.getenv("PASSWORD", DEFAULTS["password"])

    # Ensure compose ignores orphans (we spin up/down only the attestation file)
    compose_env = os.environ.copy()
    compose_env["COMPOSE_IGNORE_ORPHANS"] = "1"

    # 1) Setup .env for experiment runs
    upsert_env("EXPORT_RESULTS", "TRUE")
    upsert_env("RESULTS_DIR", args.results_dir)
    upsert_env("AUTO_START", "FALSE")
    upsert_env("COMPOSE_PROJECT_NAME", PROJECT_NAME)
    print(f"✅ .env updated: RESULTS_DIR={args.results_dir} (AUTO_START=FALSE, EXPORT_RESULTS=TRUE)")

    try:
        for run in range(1, args.runs + 1):
            print(f"\n==============================\n🏁 Run {run} / {args.runs}\n==============================")
            upsert_env("RUN_ID", run)

            # NEW: restart SECaaS so it picks up the new RUN_ID and writes secaas-run{run}.json
            restart_secaas(sudo_pass, timeout=60)

            # 2) Start ONLY attestation sidecars (robots/secaas/monitoring already running)
            print("[+] Starting attestation sidecars...")
            compose_up_attestation(compose_env)
            time.sleep(5)

            # 3) Wait for Services
            print("[+] Waiting for readiness...")
            wait_for_http(f"{MONITOR_URL}/monitor/status")
            for name, port in SERVICE_PORTS.items():
                wait_for_http(f"http://localhost:{port}/")

            # 4) Start Docker Stats
            print(f"📊 Starting docker-stats → {args.stats_dir}")
            containers = ["secaas", "robot1-sidecar", "robot2-sidecar", "robot3-sidecar", "robot4-sidecar"]
            payload = {
                "containers": containers,
                "interval": 1.0,
                "csv_dir": args.stats_dir,
                "stdout": True,
            }
            try:
                r = requests.post(f"{MONITOR_URL}/monitor/start", json=payload, timeout=10)
                print(json.dumps(r.json(), indent=2))
            except Exception as e:
                print(f"❌ Failed to start monitor: {e}")

            # 5) Start Attestations (PARALLEL)
            print("▶️  Starting attestations (Parallel)...")
            with ThreadPoolExecutor(max_workers=len(SERVICE_PORTS)) as executor:
                futures = {
                    executor.submit(trigger_request, name, port, "start"): name
                    for name, port in SERVICE_PORTS.items()
                }
                for future in as_completed(futures):
                    print(future.result())

            # 6) Run Experiment
            print(f"⏱  Running for {args.duration}s...")
            time.sleep(args.duration)

            # 7) Stop Attestations (PARALLEL)
            print("⏹  Stopping attestations (Parallel)...")
            with ThreadPoolExecutor(max_workers=len(SERVICE_PORTS)) as executor:
                futures = {
                    executor.submit(trigger_request, name, port, "stop"): name
                    for name, port in SERVICE_PORTS.items()
                }
                for future in as_completed(futures):
                    print(future.result())

            # 8) Stop Docker Stats
            print("⏹  Stopping docker-stats...")
            try:
                r = requests.post(f"{MONITOR_URL}/monitor/stop", timeout=10)
                print(json.dumps(r.json(), indent=2))
            except Exception:
                pass

            # 9) Reset Chain
            reset_chain()

            # 10) Teardown ONLY attestation sidecars
            print("🧹  Bringing down attestation sidecars...")
            compose_down_attestation(sudo_pass, compose_env)

    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")
    finally:
        cleanup(sudo_pass, compose_env)
        print(f"🎉 Done. Results in:\n  Attestations: {args.results_dir}\n  Docker stats: {args.stats_dir}")


if __name__ == "__main__":
    main()
