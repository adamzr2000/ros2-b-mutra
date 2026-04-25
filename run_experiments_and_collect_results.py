#!/usr/bin/env python3
import time
import argparse
import requests
import subprocess
import sys
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

# --- Configuration ---
MONITOR_URL      = "http://localhost:6000"
MONITOR_PORT     = 6000
ETH_MONITOR_URL  = "http://localhost:7000"
SECAAS_URL       = "http://localhost:8000"   # SECaaS is always on local machine
SECAAS_PORT      = 8000
ROBOTS_BASE_PORT = 8001   # robot1=8001, robot2=8002, ...
ETH_RPC_URL      = "http://host.docker.internal:21001"

# Multi-host defaults (overridable via CLI)
_DEFAULT_REMOTE_HOST  = "10.5.1.21"   # remote host
_DEFAULT_LOCAL_LIMIT  = 8              # robots 1-LOCAL_LIMIT are on local machine
REMOTE_USER           = "desire6g"
REMOTE_DIR            = "~/ros2-b-mutra"

# Absolute path so the script works regardless of the working directory
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# (host, port) tuple for SECaaS — always localhost
_SECAAS_HOST_PORT = ("localhost", SECAAS_PORT)


def build_sidecars(num_robots, remote_host=None, local_limit=_DEFAULT_LOCAL_LIMIT):
    """Return {name: (host, port)} for SECaaS and all robot sidecars.

    Robots 1..local_limit are on localhost (local machine).
    Robots local_limit+1..num_robots are on remote_host (remote host), if provided.
    """
    sidecars = {"secaas": ("localhost", SECAAS_PORT)}
    for i in range(1, num_robots + 1):
        port = ROBOTS_BASE_PORT + i - 1
        host = remote_host if (remote_host and i > local_limit) else "localhost"
        sidecars[f"robot{i}"] = (host, port)
    return sidecars


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

def check_all_services(sidecars):
    """Verifies that ALL defined services are up and listening (parallel)."""
    print("\n🔍 Pre-flight check: Verifying all services...")

    checks = {"monitor": f"{MONITOR_URL}/monitor/status",
              "eth-monitor": f"{ETH_MONITOR_URL}/monitor/status"}
    for name, (host, port) in sidecars.items():
        checks[name] = f"http://{host}:{port}/"

    all_ok = True
    with ThreadPoolExecutor(max_workers=len(checks)) as executor:
        futures = {executor.submit(wait_for_http, url): name for name, url in checks.items()}
        for future, name in futures.items():
            if not future.result():
                print(f"❌ Service '{name}' is not responding.")
                all_ok = False

    if all_ok:
        print("🚀 All systems GO.\n")
    return all_ok

def trigger_action(name, host_port, action, timeout=5):
    """Sends start/stop signals to the sidecar API."""
    host, port = host_port
    try:
        url = f"http://{host}:{port}/{action}"
        resp = requests.post(url, timeout=timeout)
        resp.raise_for_status()
        return f"✔ {name}: {action.upper()}"
    except Exception as e:
        return f"❌ {name}: Failed to {action} ({e})"

def get_status(name, host_port):
    """Returns the status string from /status endpoint."""
    host, port = host_port
    try:
        resp = requests.get(f"http://{host}:{port}/status", timeout=5)
        resp.raise_for_status()
        return resp.json().get("status", "unknown")
    except Exception as e:
        return f"error: {e}"

def _poll_statuses_parallel(robot_sidecars):
    """Fetches /status from all robot sidecars in parallel."""
    with ThreadPoolExecutor(max_workers=len(robot_sidecars)) as executor:
        futures = {executor.submit(get_status, name, hp): name for name, hp in robot_sidecars.items()}
        return {futures[f]: f.result() for f in as_completed(futures)}

def wait_for_all_running(robot_sidecars, timeout=30):
    """Waits until all robot sidecars have left 'idle' (i.e. report 'running'
    or already 'finished').
    """
    print(f"   ⏳ Waiting for all agents to start running (timeout={timeout}s)...")
    start = time.time()
    while time.time() - start < timeout:
        statuses = _poll_statuses_parallel(robot_sidecars)
        all_started = all(s in ("running", "finished") for s in statuses.values())
        if all_started:
            print("   ✅ All agents running.")
            return True
        pending = [n for n, s in statuses.items() if s not in ("running", "finished")]
        print(f"      Still waiting to start: {pending}")
        time.sleep(0.5)
    print("   ❌ Timeout: not all agents started in time.")
    return False

def wait_for_all_finished(robot_sidecars, timeout=120):
    """Polls all robot sidecars until all report 'finished' or timeout."""
    print(f"   ⏳ Waiting for all agents to finish (timeout={timeout}s)...")
    start = time.time()
    while time.time() - start < timeout:
        statuses = _poll_statuses_parallel(robot_sidecars)
        all_done = all(s == "finished" for s in statuses.values())
        if all_done:
            print("   ✅ All agents finished.")
            return True
        pending = [n for n, s in statuses.items() if s != "finished"]
        print(f"      Still waiting: {pending}")
        time.sleep(2)
    print("   ❌ Timeout: not all agents finished in time.")
    return False

def set_config(name, host_port, payload: dict):
    """Sends a partial config update to a single sidecar via POST /config."""
    host, port = host_port
    try:
        resp = requests.post(f"http://{host}:{port}/config", json=payload, timeout=5)
        resp.raise_for_status()
        applied = resp.json().get("applied", payload)
        return f"✔ {name}: {applied}"
    except Exception as e:
        return f"❌ {name}: Failed to set config ({e})"

def set_config_all(robot_sidecars, payload: dict):
    """Sends a partial config update to all robot sidecars in parallel.
    SECaaS is excluded — it has no /config endpoint and is always event-driven.
    """
    print(f"   [Config] Pushing to all robot sidecars: {payload}...")
    with ThreadPoolExecutor(max_workers=len(robot_sidecars)) as executor:
        futures = [executor.submit(set_config, n, hp, payload) for n, hp in robot_sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

def stop_all_agents(sidecars):
    """Stops all agents in parallel."""
    print("   [Agents] Stopping...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, hp, "stop", 15) for n, hp in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

def reset_chain():
    """Resets the blockchain state via SECaaS API."""
    try:
        requests.post(f"{SECAAS_URL}/reset", timeout=10)
        print("   [Chain] State reset successfully.")
    except Exception as e:
        print(f"   [Chain] ⚠️ Warning: Reset failed: {e}")


def run_experiment_loop(run_id, duration, stats_dir, attestation_dir, sidecars, robot_sidecars,
                        startup_mode=False, remote_monitor_url=None, blockchain_dir=None):
    print(f"\n==============================\n🏁 Run {run_id}\n==============================")

    set_config_all(robot_sidecars, {"one_shot": startup_mode, "results_dir": attestation_dir})
    set_config("secaas", _SECAAS_HOST_PORT, {"results_dir": attestation_dir})

    # 2. Start Docker Monitor(s)
    print(f"   [Monitor] Starting stats -> {stats_dir}")
    local_containers  = ["secaas"] + [f"{r}-sidecar" for r, hp in robot_sidecars.items() if hp[0] == "localhost"]
    remote_containers = [f"{r}-sidecar" for r, hp in robot_sidecars.items() if hp[0] != "localhost"]
    try:
        requests.post(f"{MONITOR_URL}/monitor/start", json={
            "containers": local_containers,
            "interval": 1.0,
            "csv_dir": stats_dir,
            "run_id": run_id
        }, timeout=5)
    except Exception as e:
        print(f"   ⚠️ Local monitor start failed: {e}")
    if remote_monitor_url and remote_containers:
        try:
            requests.post(f"{remote_monitor_url}/monitor/start", json={
                "containers": remote_containers,
                "interval": 1.0,
                "csv_dir": stats_dir,
                "run_id": run_id
            }, timeout=5)
        except Exception as e:
            print(f"   ⚠️ Remote monitor start failed: {e}")

    # Start blockchain monitor (continuous mode only)
    if not startup_mode and blockchain_dir:
        print(f"   [Eth Monitor] Starting blockchain stats -> {blockchain_dir}")
        try:
            requests.post(f"{ETH_MONITOR_URL}/monitor/start", json={
                "rpc_url": ETH_RPC_URL,
                "poll_interval": 1.0,
                "csv_dir": blockchain_dir,
                "csv_name": f"blockchain-run{run_id}",
            }, timeout=5)
        except Exception as e:
            print(f"   ⚠️ Eth monitor start failed: {e}")

    # Allow monitor to complete one clean baseline sample before agents start
    time.sleep(1)

    # 3. Start Agents (Parallel)
    print("   [Agents] Starting...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, hp, "start") for n, hp in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    # 4. Wait until all robots have left 'idle'
    wait_for_all_running(robot_sidecars, timeout=30)

    if startup_mode:
        wait_for_all_finished(robot_sidecars, timeout=180)
        stop_all_agents(sidecars)
        time.sleep(3)
        reset_chain()
    else:
        print(f"   ⏳ Running for {duration}s...")
        time.sleep(duration)
        stop_all_agents(sidecars)
        time.sleep(2)

    # Stop Monitor(s) — allow up to 60s for parallel container shutdown
    try:
        requests.post(f"{MONITOR_URL}/monitor/stop", timeout=60)
    except Exception:
        pass
    if remote_monitor_url:
        try:
            requests.post(f"{remote_monitor_url}/monitor/stop", timeout=60)
        except Exception:
            pass
    if not startup_mode and blockchain_dir:
        try:
            requests.post(f"{ETH_MONITOR_URL}/monitor/stop", timeout=10)
        except Exception:
            pass


def run_continuous_warmup(sidecars, robot_sidecars):
    """One-shot warmup run with export disabled to seed lastSuccess on-chain."""
    print("\n🔥 Continuous warmup: seeding lastSuccess via one-shot run (export disabled)...")

    set_config_all(robot_sidecars, {"one_shot": True, "export_enabled": False})
    set_config("secaas", _SECAAS_HOST_PORT, {"export_enabled": False})

    print("   [Warmup] Starting agents...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, hp, "start") for n, hp in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    wait_for_all_running(robot_sidecars, timeout=30)
    wait_for_all_finished(robot_sidecars, timeout=180)
    stop_all_agents(sidecars)

    print("   ✅ Warmup complete. lastSuccess seeded for all robots.\n")


def reset_checkpoints(remote_host=None, local_limit=_DEFAULT_LOCAL_LIMIT, num_robots=0):
    """Clears all event watcher checkpoint JSON files (local + remote if multi-host)."""
    base = os.path.join(_SCRIPT_DIR, "checkpoints")
    print("   [Checkpoints] Clearing local event watcher state...")
    for entry in os.scandir(base):
        if entry.is_dir():
            subprocess.run(
                ["find", entry.path, "-type", "f", "-name", "*.json", "-delete"],
                check=True,
            )
    if remote_host and num_robots > local_limit:
        print(f"   [Checkpoints] Clearing remote event watcher state on {remote_host}...")
        subprocess.run(
            ["ssh", f"{REMOTE_USER}@{remote_host}",
             f"find {REMOTE_DIR}/checkpoints -type f -name '*.json' -delete 2>/dev/null || true"],
            check=True,
        )
    print("   [Checkpoints] Done.")


def sync_remote_results(remote_host):
    """Rsync only results/ subdirs from remote host, never overwriting existing files."""
    for subdir in ["attestation-times/results", "docker-stats/results"]:
        local_dst  = os.path.join(_SCRIPT_DIR, "experiments", "data", subdir) + "/"
        remote_src = f"{REMOTE_USER}@{remote_host}:{REMOTE_DIR}/experiments/data/{subdir}/"
        print(f"\n📥 Syncing {remote_src} → {local_dst}")
        os.makedirs(local_dst, exist_ok=True)
        subprocess.run(
            ["rsync", "-av", "--ignore-existing", remote_src, local_dst],
            check=True,
        )
    print("✅ Remote results synced.")


def run_experiments(runs, duration, stats_dir, attestation_dir, sidecars, robot_sidecars,
                    startup_mode, remote_host=None, local_limit=_DEFAULT_LOCAL_LIMIT,
                    num_robots=0, remote_monitor_url=None, blockchain_dir=None):
    reset_checkpoints(remote_host=remote_host, local_limit=local_limit, num_robots=num_robots)

    # Pre-set SECaaS results_dir immediately so any stale blockchain events from a
    # previous session land in the correct directory, not the old one.
    set_config("secaas", _SECAAS_HOST_PORT, {"results_dir": attestation_dir})

    if not startup_mode:
        run_continuous_warmup(sidecars, robot_sidecars)
        set_config_all(robot_sidecars, {"export_enabled": True})
        set_config("secaas", _SECAAS_HOST_PORT, {"export_enabled": True})
        print("   [System] Cooling down for 5s after warmup...")
        time.sleep(5)

    for i in range(1, runs + 1):
        run_experiment_loop(i, duration, stats_dir, attestation_dir, sidecars, robot_sidecars,
                            startup_mode=startup_mode, remote_monitor_url=remote_monitor_url,
                            blockchain_dir=blockchain_dir)
        if i < runs:
            print("   [System] Cooling down for 5s...")
            time.sleep(5)


# --- Main ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Data collection — two modes: startup (one-shot) or continuous (fixed duration)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--robots",      type=int, default=4,
                        help="Number of robot sidecars (default: 4, ports 8001..800N)")
    parser.add_argument("--runs",        type=int, default=1,
                        help="Number of runs (default: 1)")
    parser.add_argument("--remote-host", default=None,
                        help=f"IP of remote host for robots beyond --local-limit "
                             f"(default: {_DEFAULT_REMOTE_HOST} when N > local-limit)")
    parser.add_argument("--local-limit", type=int, default=_DEFAULT_LOCAL_LIMIT,
                        help=f"Robots 1-N are on localhost; rest are on --remote-host "
                             f"(default: {_DEFAULT_LOCAL_LIMIT})")

    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--startup",  action="store_true",
                      help="One-shot mode: poll until all agents report 'finished'")
    mode.add_argument("--duration", type=int,
                      help="Continuous mode: run for this many seconds then stop agents")

    args = parser.parse_args()

    # Auto-enable remote host for multi-host deployments
    remote_host = args.remote_host
    if remote_host is None and args.robots > args.local_limit:
        remote_host = _DEFAULT_REMOTE_HOST
        print(f"ℹ️  N={args.robots} > local-limit={args.local_limit}: "
              f"using remote host {remote_host} for robots {args.local_limit + 1}–{args.robots}")

    remote_monitor_url = f"http://{remote_host}:{MONITOR_PORT}" if remote_host else None

    mode_label  = "startup" if args.startup else "continuous"
    robot_label = f"N{args.robots}"
    stats_dir       = f"/experiments/data/docker-stats/results/{robot_label}/{mode_label}"
    attestation_dir = f"/experiments/data/attestation-times/results/{robot_label}/{mode_label}"
    blockchain_dir  = f"/experiments/data/blockchain-stats/results/{robot_label}/continuous" if not args.startup else None

    sidecars = build_sidecars(args.robots, remote_host=remote_host, local_limit=args.local_limit)
    robot_sidecars = {k: v for k, v in sidecars.items() if k != "secaas"}

    print(f"📋 Configured sidecars:")
    for name, (host, port) in sidecars.items():
        print(f"   {name}: http://{host}:{port}")
    if args.startup:
        print("🔁 Mode: STARTUP (one-shot)")
    else:
        print(f"🔁 Mode: CONTINUOUS ({args.duration}s per run)")
    print(f"📁 Stats dir:       {stats_dir}")
    print(f"📁 Attestation dir: {attestation_dir}")
    if blockchain_dir:
        print(f"📁 Blockchain dir:  {blockchain_dir}")

    if remote_monitor_url:
        print(f"   monitor (remote): {remote_monitor_url}")

    if not check_all_services(sidecars):
        print("🛑 Aborting: Not all services are ready.")
        sys.exit(1)

    if remote_monitor_url and not wait_for_http(f"{remote_monitor_url}/monitor/status"):
        print("⚠️  Remote monitor not responding — remote sidecar stats won't be collected.")
        remote_monitor_url = None

    try:
        run_experiments(args.runs, args.duration or 0, stats_dir, attestation_dir,
                        sidecars, robot_sidecars, startup_mode=args.startup,
                        remote_host=remote_host, local_limit=args.local_limit,
                        num_robots=args.robots, remote_monitor_url=remote_monitor_url,
                        blockchain_dir=blockchain_dir)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")

    print(f"\n🎉 All runs completed. Results saved to {stats_dir}")

    if remote_host:
        sync_remote_results(remote_host)

    # --- Cleanup Ghost Artifacts ---
    # Remove all secaas.json files anywhere under the attestation results tree.
    # SECaaS exports its own JSON on every run; it's not useful for analysis.
    results_root = os.path.join(_SCRIPT_DIR, "experiments", "data", "attestation-times", "results")
    result = subprocess.run(
        ["find", results_root, "-type", "f", "-name", "secaas.json", "-delete", "-print"],
        capture_output=True, text=True,
    )
    if result.stdout.strip():
        for f in result.stdout.strip().splitlines():
            print(f"🧹 Removed ghost artifact: {f}")
