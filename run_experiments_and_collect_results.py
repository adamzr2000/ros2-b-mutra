#!/usr/bin/env python3
import time
import argparse
import requests
import subprocess
import sys
import os
from concurrent.futures import ThreadPoolExecutor, as_completed

# --- Configuration ---
MONITOR_URL = "http://localhost:6000"
SECAAS_URL  = "http://localhost:8000"
SECAAS_PORT = 8000
ROBOTS_BASE_PORT = 8001  # robot1=8001, robot2=8002, ...

# Absolute path so the script works regardless of the working directory
_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def build_sidecars(num_robots):
    """Builds the sidecar map dynamically based on number of robots.
    SECaaS is always at 8000, robots start at 8001.
    """
    sidecars = {"secaas": SECAAS_PORT}
    for i in range(1, num_robots + 1):
        sidecars[f"robot{i}"] = ROBOTS_BASE_PORT + i - 1
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

    checks = {"monitor": f"{MONITOR_URL}/monitor/status"}
    for name, port in sidecars.items():
        checks[name] = f"http://localhost:{port}/"

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

def trigger_action(name, port, action, timeout=5):
    """Sends start/stop signals to the sidecar API."""
    try:
        url = f"http://localhost:{port}/{action}"
        resp = requests.post(url, timeout=timeout)
        resp.raise_for_status()
        return f"✔ {name}: {action.upper()}"
    except Exception as e:
        return f"❌ {name}: Failed to {action} ({e})"

def get_status(name, port):
    """Returns the status string from /status endpoint."""
    try:
        resp = requests.get(f"http://localhost:{port}/status", timeout=5)
        resp.raise_for_status()
        return resp.json().get("status", "unknown")
    except Exception as e:
        return f"error: {e}"

def _poll_statuses_parallel(robot_sidecars):
    """Fetches /status from all robot sidecars in parallel."""
    with ThreadPoolExecutor(max_workers=len(robot_sidecars)) as executor:
        futures = {executor.submit(get_status, name, port): name for name, port in robot_sidecars.items()}
        return {futures[f]: f.result() for f in as_completed(futures)}

def wait_for_all_running(robot_sidecars, timeout=30):
    """Waits until all robot sidecars have left 'idle' (i.e. report 'running'
    or already 'finished').

    After /stop, sidecars return 'idle' (lastResult='stopped').  The transition
    on /start is: idle → running → finished (ONE_SHOT) or idle → running
    (continuous).  We accept 'finished' as a valid "has started" state so that
    a very fast attestation cycle (completed before the first 0.5 s poll) does
    not cause a 30 s timeout here before wait_for_all_finished is reached.
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
    """Polls all robot sidecars until all report 'finished' or timeout.
    Excludes secaas since it has no oneshot/status concept.
    """
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

def set_config(name, port, payload: dict):
    """Sends a partial config update to a single robot sidecar via POST /config."""
    try:
        resp = requests.post(f"http://localhost:{port}/config", json=payload, timeout=5)
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
        futures = [executor.submit(set_config, n, p, payload) for n, p in robot_sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

def stop_all_agents(sidecars):
    """Stops all agents in parallel. Uses a longer timeout than other actions
    because /stop blocks while draining worker goroutines (up to 10s in Go)."""
    print("   [Agents] Stopping...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, p, "stop", 15) for n, p in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

def reset_chain():
    """Resets the blockchain state via SECaaS API (calls ResetChain() on the
    contract, restricted to the SECaaS address).

    What the contract actually resets:
      - attestationChain[]  — completed attestation ID history wiped
      - rrIndex             — round-robin verifier election counter reset to 0
      - lastSuccess[addr]   — freshness timestamps reset to 0 for all participants

    What it does NOT touch:
      - agent[addr].registered / agent[addr].name — agents stay registered
      - participants[]                             — participant list preserved

    Consequence for startup mode: after each reset, lastSuccess=0 for
    everyone, so ElectVerifier finds no fresh candidates and always returns
    SECaaS as verifier. Every one-shot run therefore uses SECaaS as verifier
    and is resolved via ResolveAttestationSECaaS (not CloseAttestationProcess).

    Not needed in continuous mode where chain state accumulates intentionally.
    The sleep(3) before calling this allows agents to finish any in-flight
    blockchain writes before the reset fires.
    """
    try:
        requests.post(f"{SECAAS_URL}/reset", timeout=10)
        print("   [Chain] State reset successfully.")
    except Exception as e:
        print(f"   [Chain] ⚠️ Warning: Reset failed: {e}")


def run_experiment_loop(run_id, duration, stats_dir, attestation_dir, sidecars, robot_sidecars, startup_mode=False):
    print(f"\n==============================\n🏁 Run {run_id}\n==============================")

    # 1. Push mode + results_dir to all sidecars before anything else.
    #    results_dir is set every run so each sidecar writes to the correct
    #    sub-directory regardless of what the env var was set to at startup.
    #    SECaaS gets only results_dir (no one_shot concept).
    set_config_all(robot_sidecars, {"one_shot": startup_mode, "results_dir": attestation_dir})
    set_config("secaas", SECAAS_PORT, {"results_dir": attestation_dir})

    # 2. Start Docker Monitor
    print(f"   [Monitor] Starting stats -> {stats_dir}")
    try:
        requests.post(f"{MONITOR_URL}/monitor/start", json={
            "containers": ["secaas"] + [f"{r}-sidecar" for r in robot_sidecars],
            "interval": 1.0,
            "csv_dir": stats_dir,
            "run_id": run_id
        }, timeout=5)
    except Exception as e:
        print(f"   ⚠️ Monitor start failed: {e}")

    # 3. Start Agents (Parallel) — fires /start on all sidecars simultaneously
    print("   [Agents] Starting...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, p, "start") for n, p in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    # 4. Wait until all robots have left 'idle' (running or finished).
    #    Ensures /start has been fully processed before we start timing or
    #    polling for completion.  In startup mode, also guards against
    #    proceeding to wait_for_all_finished while sidecars are still
    #    transitioning from idle.
    wait_for_all_running(robot_sidecars, timeout=30)

    if startup_mode:
        # 5. Poll until all robot sidecars report "finished" (self-stop after one attestation)
        wait_for_all_finished(robot_sidecars, timeout=120)

        # 6. Explicitly stop all agents to reset sidecar state (measuring=false,
        #    ResultsFile cleared) so the next run starts clean
        stop_all_agents(sidecars)

        # 7. Allow agents to finish any in-flight blockchain writes, then reset chain
        #    so the next run starts with clean agent registration and no stale IDs
        time.sleep(3)
        reset_chain()
    else:
        # 5. Wait fixed duration then stop explicitly (continuous mode).
        #    Chain state accumulates intentionally across runs in this mode.
        print(f"   ⏳ Running for {duration}s...")
        time.sleep(duration)

        # 6. Stop Agents (Parallel)
        stop_all_agents(sidecars)

        # 7. Brief cooldown so the monitor captures the final stats window
        #    before the stop signal is sent (monitor collects at 1s intervals).
        time.sleep(2)

    # 8. Stop Monitor
    try:
        requests.post(f"{MONITOR_URL}/monitor/stop", timeout=5)
    except Exception:
        pass


def run_continuous_warmup(sidecars, robot_sidecars):
    """Runs a single one-shot attestation cycle with export disabled so that
    every robot gets a non-zero lastSuccess timestamp on the contract before
    the real data-collection loop starts.  Without this, ElectVerifier finds
    no fresh candidates and always falls back to SECaaS as verifier, which
    is not representative of steady-state continuous operation.

    The warmup:
      - disables export on all sidecars and SECaaS (no files written)
      - sets one_shot=True so each robot self-stops after one attestation
      - skips the docker monitor (no stats recorded)
      - does NOT reset the chain afterward (that would undo lastSuccess)
    """
    print("\n🔥 Continuous warmup: seeding lastSuccess via one-shot run (export disabled)...")

    # Disable export everywhere for the warmup
    set_config_all(robot_sidecars, {"one_shot": True, "export_enabled": False})
    set_config("secaas", SECAAS_PORT, {"export_enabled": False})

    # Start all agents
    print("   [Warmup] Starting agents...")
    with ThreadPoolExecutor(max_workers=len(sidecars)) as executor:
        futures = [executor.submit(trigger_action, n, p, "start") for n, p in sidecars.items()]
        for f in as_completed(futures):
            print(f"      {f.result()}")

    wait_for_all_running(robot_sidecars, timeout=30)
    wait_for_all_finished(robot_sidecars, timeout=120)
    stop_all_agents(sidecars)

    print("   ✅ Warmup complete. lastSuccess seeded for all robots.\n")


def reset_checkpoints():
    """Clears all event watcher checkpoint JSON files on the host before an
    experiment batch starts.  Without this, a watcher resumed from a stale
    from_block may scan many old blocks before finding new events, adding
    latency noise to timing measurements in run 1.
    """
    base = os.path.join(_SCRIPT_DIR, "checkpoints")
    print("   [Checkpoints] Clearing event watcher state...")
    for entry in os.scandir(base):
        if entry.is_dir():
            subprocess.run(
                ["find", entry.path, "-type", "f", "-name", "*.json", "-delete"],
                check=True,
            )
    print("   [Checkpoints] Done.")


def run_experiments(runs, duration, stats_dir, attestation_dir, sidecars, robot_sidecars, startup_mode):
    reset_checkpoints()

    if not startup_mode:
        run_continuous_warmup(sidecars, robot_sidecars)
        # Re-enable export on all sidecars and SECaaS before the real loop
        set_config_all(robot_sidecars, {"export_enabled": True})
        set_config("secaas", SECAAS_PORT, {"export_enabled": True})
        print("   [System] Cooling down for 5s after warmup...")
        time.sleep(5)

    for i in range(1, runs + 1):
        run_experiment_loop(i, duration, stats_dir, attestation_dir, sidecars, robot_sidecars, startup_mode=startup_mode)
        if i < runs:
            print("   [System] Cooling down for 5s...")
            time.sleep(5)


# --- Main ---
if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Data collection — two modes: startup (one-shot) or continuous (fixed duration)",
        formatter_class=argparse.RawTextHelpFormatter,
    )
    parser.add_argument("--robots",   type=int, default=4, help="Number of robot sidecars (default: 4, ports 8001..800N)")
    parser.add_argument("--runs",     type=int, default=1, help="Number of runs (default: 1)")

    # Mutually exclusive: either one-shot startup mode or fixed-duration continuous mode
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--startup",  action="store_true", help="One-shot mode: poll until all agents report 'finished' then stop explicitly")
    mode.add_argument("--duration", type=int,            help="Continuous mode: run for this many seconds then stop agents")

    args = parser.parse_args()

    # Derive output directories from robot count and mode
    mode_label = "startup" if args.startup else "continuous"
    robot_label = f"N{args.robots}"
    stats_dir       = f"/experiments/data/docker-stats/results/{robot_label}/{mode_label}"
    attestation_dir = f"/experiments/data/attestation-times/results/{robot_label}/{mode_label}"

    # Build sidecar map dynamically
    sidecars = build_sidecars(args.robots)
    robot_sidecars = {k: v for k, v in sidecars.items() if k != "secaas"}

    print(f"📋 Configured sidecars: { {k: v for k, v in sidecars.items()} }")
    if args.startup:
        print("🔁 Mode: STARTUP (one-shot)")
    else:
        print(f"🔁 Mode: CONTINUOUS ({args.duration}s per run)")
    print(f"📁 Stats dir:       {stats_dir}")
    print(f"📁 Attestation dir: {attestation_dir}")

    # 1. Pre-flight: wait for all services
    if not check_all_services(sidecars):
        print("🛑 Aborting: Not all services are ready.")
        sys.exit(1)

    # 2. Run experiments (args.duration is None for --startup; pass 0 as safe default)
    try:
        run_experiments(args.runs, args.duration or 0, stats_dir, attestation_dir, sidecars, robot_sidecars, startup_mode=args.startup)
    except KeyboardInterrupt:
        print("\n🛑 Interrupted by user.")

    print(f"\n🎉 All runs completed. Results saved to {stats_dir}")

    # --- Cleanup Ghost Artifacts ---
    # secaas.json (no run number) is created when SECaaS receives /start before
    # /config has set a results_dir, so _next_run_json hasn't run yet and the
    # file lands as a bare "secaas.json".  It contains no useful data.
    ghost_file = os.path.join(attestation_dir, "secaas.json")
    if os.path.exists(ghost_file):
        try:
            os.remove(ghost_file)
            print(f"🧹 Cleaned up artifact: {ghost_file}")
        except PermissionError:
            print(f"⚠️ Permission denied deleting {ghost_file}. Try: sudo rm -f {ghost_file}")
        except OSError as e:
            print(f"⚠️ Failed deleting {ghost_file}: {e}")
