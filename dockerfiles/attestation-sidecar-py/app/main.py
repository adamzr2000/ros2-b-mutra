import os
import re
import glob
import json
import threading

from fastapi import FastAPI

from app.internal.logger import init_logging, info, warn, error, debug, green_text
from app.internal.utils import helpers
from app.internal.blockchain.client import BlockchainClient
from app.internal.attestation.prover import run_prover_logic_continuous_mode
from app.internal.attestation.verifier import run_verifier_logic_sequential
from app.internal.storage.redis import RedisStorageBackend
from app.internal.storage.memory import MemoryStorageBackend

# -----------------------------------------------------------------------------
# Logging init (custom logger)
# -----------------------------------------------------------------------------
init_logging()

app = FastAPI()

# -----------------------------------------------------------------------------
# Helpers
# -----------------------------------------------------------------------------
def _next_run_json(results_dir: str, participant: str) -> str:
    os.makedirs(results_dir, exist_ok=True)
    base = results_dir.rstrip("/")
    safe = participant  # names already controlled; adjust if needed
    pattern = os.path.join(base, f"{safe}-run*.json")
    existing = glob.glob(pattern)
    max_idx = 0
    rx = re.compile(rf"{re.escape(safe)}-run(\d+)\.json$")
    for p in existing:
        m = rx.search(os.path.basename(p))
        if m:
            try:
                idx = int(m.group(1))
                if idx > max_idx:
                    max_idx = idx
            except ValueError:
                pass
    return os.path.join(base, f"{safe}-run{max_idx+1}.json")

def start_attestation(app: FastAPI):
    # Create stop event / threads list if not present (idempotent)
    if not hasattr(app.state, "stop_event"):
        app.state.stop_event = threading.Event()
    if not hasattr(app.state, "threads"):
        app.state.threads = []

    # If we’re restarting, make sure stop is cleared
    app.state.stop_event.clear()

    if app.state.export_enabled:
        # Ensure per-run JSON path is set, then init file
        if not getattr(app.state, "results_file", None):
            # Fallback only (should rarely happen now)
            app.state.results_file = _next_run_json(app.state.results_dir, app.state.participant_name)
            info(f"Results file selected (fallback): {app.state.results_file}")

        helpers.ensure_results_initialized(
            app.state.results_dir,
            app.state.participant_name,
            json_path=app.state.results_file
        )

    if app.state.measuring:
        info("Agent attestation already running; ignoring start request")
        return

    app.state.measuring = True
    info("Agent attestation loop started")

    t1 = threading.Thread(
        target=run_prover_logic_continuous_mode,
        args=(app, app.state.stop_event),
        name=app.state.participant_name,
        daemon=True
    )
    t2 = threading.Thread(
        target=run_verifier_logic_sequential,
        args=(app, app.state.stop_event),
        name=app.state.participant_name,
        daemon=True
    )

    t1.start()
    t2.start()
    app.state.threads.extend([t1, t2])

def stop_attestation(app: FastAPI):
    # Stop prover measuring background
    app.state.measuring = False

    # Signal workers to stop
    if hasattr(app.state, "stop_event"):
        app.state.stop_event.set()

    # Best-effort join of all tracked threads
    for t in getattr(app.state, "threads", []):
        try:
            t.join(timeout=3)
        except Exception as e:
            warn(f"Failed joining thread {t.name}: {e}")

    # Reset the thread list after stopping
    if hasattr(app.state, "threads"):
        app.state.threads.clear()

    if app.state.export_enabled:
        helpers.mark_experiment_stop(
            app.state.results_dir, 
            app.state.participant_name, 
            json_path=getattr(app.state, "results_file", None)
        )

# -----------------------------------------------------------------------------
# Startup
# -----------------------------------------------------------------------------
@app.on_event("startup")
async def startup():    
    config_path  = helpers.get_env_str("CONFIG_PATH", default="")
    if not config_path:
        error("CONFIG_PATH environment variable is required.")
        raise SystemExit(1)

    try:
        with open(config_path, "r") as json_file:
            data = json.load(json_file)

        eth_address = data["eth_address"]
        private_key = data["private_key"]
        eth_node_url = data["eth_node_url"]
        contract_address = data["contract_address"]

        # optional compute/hash params
        if 'cmd_name' in data:
            app.state.cmd_name = data['cmd_name']
        if 'text_section_size' in data:
            app.state.text_section_size = int(data['text_section_size'])
        if 'offset' in data:
            app.state.offset = int(data['offset'])
            
    except Exception as e:
        error(f"Configuration error: {str(e)}")
        raise SystemExit(1)

    # App state
    app.state.participant = "agent"
    app.state.participant_name = data.get("name", "agent")
    app.state.export_enabled = os.getenv("EXPORT_RESULTS", "false").lower() == "true"
    app.state.results_dir = helpers.get_env_str("RESULTS_DIR", default="/experiments/data/attestation-times")
    contract_abi = data.get("contract_abi")

    if not contract_abi:
        error("Missing 'contract_abi' in config JSON.")
        raise SystemExit(1)

    # Blockchain client
    app.state.blockchain_client = BlockchainClient(
        eth_address=eth_address,
        private_key=private_key,
        eth_node_url=eth_node_url,
        contract_address=contract_address,
        contract_abi=contract_abi
    )

    app.state.wait_for_tx_confirmations = os.getenv("WAIT_FOR_TX_CONFIRMATIONS", "false").lower() == "true"

    app.state.active_attestations = {}   # attestation_id → Thread

    app.state.fail_attestation_flag = os.getenv("FAIL_ATTESTATION", "false").lower() == "true"

    # Events watcher config
    app.state.event_confirmations = helpers.get_env_int("EVENT_CONFIRMATIONS", default=1)
    app.state.event_batch_size = helpers.get_env_int("EVENT_BATCH_SIZE", default=1000)
    app.state.event_poll_interval = helpers.get_env_float("EVENT_POLL_INTERVAL", default=1)
    app.state.event_lookback_blocks = helpers.get_env_int("EVENT_LOOKBACK_BLOCKS", default=0)
    app.state.event_checkpoint_dir = helpers.get_env_str("EVENT_CHECKPOINT_DIR", default="/checkpoints")
    
    # Chain display config
    app.state.chain_display = helpers.get_env_str("CHAIN_DISPLAY", default="false").strip().lower() in ("true", "yes", "1")
    app.state.chain_display_sec = helpers.get_env_int("CHAIN_DISPLAY_SEC", default=30)
    app.state.chain_display_n   = helpers.get_env_int("CHAIN_DISPLAY_N", default=10)
    
    # Thread tracking
    app.state.stop_event = threading.Event()
    app.state.threads = []

    # Storage
    use_redis = os.getenv("USE_REDIS", "false").lower() == "true"
    app.state.storage = RedisStorageBackend() if use_redis else MemoryStorageBackend()
    app.state.memory_storage_file = helpers.get_env_str("MEMORY_STORAGE_FILE", default="")

    # Prover config
    app.state.prover_threshold = helpers.get_env_int("PROVER_THRESHOLD", default=300)
    app.state.measuring = False
    app.state.digests = []
    app.state.digests_count = 0
    app.state.final_digest = ""

    auto_start = os.getenv("AUTO_START", "false").strip().lower() == "true"

    # Precompute results file
    app.state.results_file = None
    if app.state.export_enabled:
        run_id = os.getenv("RUN_ID", "").strip()
        if run_id:
            app.state.results_file = os.path.join(app.state.results_dir, f"{app.state.participant_name}-run{run_id}.json")
            info(f"Preselected results file (RUN_ID={run_id}): {app.state.results_file}")
        else:
            app.state.results_file = _next_run_json(app.state.results_dir, app.state.participant_name)
            info(f"Preselected results file (auto): {app.state.results_file}")

    # Background roles: chain display or register + maybe autostart
    if app.state.chain_display:
        t = threading.Thread(
            target=app.state.blockchain_client.periodic_attestation_chain_display,
            args=(app.state.chain_display_n, app.state.chain_display_sec),
            name="ChainDisplay",
            daemon=True
        )
        t.start()
        app.state.threads.append(t)
    else:
        if not app.state.blockchain_client.is_registered():
            info(f"Registering agent '{app.state.participant_name}' ({eth_address}) ...")
            tx_hash = app.state.blockchain_client.register_agent(app.state.participant_name, wait=True, timeout=30)
            info(f"{app.state.participant_name} registered (Tx: {tx_hash})")
        
        if auto_start:
            start_attestation(app)

    info(green_text("Startup completed for participant '%s'"), app.state.participant_name)

# -----------------------------------------------------------------------------
# API routes
# -----------------------------------------------------------------------------
@app.get("/")
def root_endpoint():
    return {"message": "Attestation sidecar running."}

@app.post("/start")
def start_endpoint():
    start_attestation(app)
    return {"message": "Attestation started."}

@app.post("/stop")
def stop_endpoint():
    stop_attestation(app)
    return {"message": "Attestation stopped."}

# -----------------------------------------------------------------------------
# Shutdown
# -----------------------------------------------------------------------------
@app.on_event("shutdown")
async def shutdown():
    stop_attestation(app)
    # If you later add a close() to BlockchainClient, it'll be called here safely.
    getattr(app.state.blockchain_client, "close", lambda: None)()