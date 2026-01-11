import logging
import json
import utils
from blockchain_manager import BlockchainManager

import os
import threading
import glob, re
from web3 import Web3
from fastapi import FastAPI
from fastapi.responses import JSONResponse

from oracle_verifier import run_secaas_logic

# === ANSI + Logging (same as you have) ===
GREEN = "\033[92m"; YELLOW = "\033[93m"; BLUE = "\033[94m"; MAGENTA = "\033[95m"; CYAN = "\033[96m"; RESET = "\033[0m"

# === Logging ===
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()

if LOG_LEVEL == "NONE":
    print("[logging] LOG_LEVEL=NONE → all logs disabled")
    logging.disable(logging.CRITICAL)  # silence everything
else:
    numeric_level = getattr(logging, LOG_LEVEL, logging.INFO)
    logging.basicConfig(format="%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s", level=numeric_level)
    print(f"[logging] LOG_LEVEL={LOG_LEVEL} ({numeric_level})")

app = FastAPI()
logger = logging.getLogger(__name__)

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
            logger.info(f"Results file selected (fallback): {app.state.results_file}")

        utils.ensure_results_initialized(
            app.state.results_dir,
            app.state.participant_name,
            json_path=app.state.results_file
        )

    logger.info("SECaaS attestation loop started")
    t = threading.Thread(target=run_secaas_logic, args=(app, app.state.stop_event), name="SECaaS", daemon=True)
    t.start()
    app.state.threads.append(t)

def stop_attestation(app: FastAPI):

    # Signal workers to stop
    if hasattr(app.state, "stop_event"):
        app.state.stop_event.set()

    # Best-effort join of all tracked threads
    for t in getattr(app.state, "threads", []):
        try:
            t.join(timeout=3)
        except Exception as e:
            logger.warning(f"Failed joining thread {t.name}: {e}")

    # Reset the thread list after stopping
    if hasattr(app.state, "threads"):
        app.state.threads.clear()

    if app.state.export_enabled:
        utils.mark_experiment_stop(app.state.results_dir, app.state.participant_name, json_path=getattr(app.state, "results_file", None))

@app.on_event("startup")
async def startup():
    JSON_FILE_PATH = utils.get_env_str("CONFIG_PATH", default="")
    if not JSON_FILE_PATH:
        logger.error("CONFIG_PATH environment variable is required.")
        raise SystemExit(1)

    try:
        with open(JSON_FILE_PATH, "r") as json_file:
            data = json.load(json_file)
        eth_address = data["eth_address"]
        private_key = data["private_key"]
        eth_node_url = data["eth_node_url"]
        contract_address = Web3.toChecksumAddress(data["contract_address"])
    except Exception as e:
        logger.error(f"Configuration error: {str(e)}")
        raise SystemExit(1)

    # App state
    app.state.participant = "secaas"
    app.state.participant_name = "secaas"

    app.state.blockchain_manager = BlockchainManager(
        eth_address=eth_address,
        private_key=private_key,
        eth_node_url=eth_node_url,
        abi_path="/smart-contracts/artifacts/contracts/MasMutualAttestation.sol/MasMutualAttestation.json",
        contract_address=contract_address
    )
    app.state.active_attestations = {}   # attestation_id → Thread
    app.state.export_enabled = os.getenv("EXPORT_RESULTS", "false").lower() == "true"
    app.state.results_dir = utils.get_env_str("RESULTS_DIR", default="/experiments/data/attestation-times")
    app.state.fail_attestation_flag = os.getenv("FAIL_ATTESTATION", "false").lower() == "true"
    app.state.event_confirmations = utils.get_env_int("EVENT_CONFIRMATIONS", default=1)
    app.state.event_batch_size = utils.get_env_int("EVENT_BATCH_SIZE", default=1000)
    app.state.event_poll_interval = utils.get_env_float("EVENT_POLL_INTERVAL", default=1)
    app.state.event_lookback_blocks = utils.get_env_int("EVENT_LOOKBACK_BLOCKS", default=0)
    app.state.event_checkpoint_dir = utils.get_env_str("EVENT_CHECKPOINT_DIR", default="/checkpoints")
    app.state.chain_display = utils.get_env_str("CHAIN_DISPLAY", default="false").strip().lower() in ("true", "yes", "1")
    app.state.chain_display_sec = utils.get_env_int("CHAIN_DISPLAY_SEC", default=30)
    app.state.chain_display_n   = utils.get_env_int("CHAIN_DISPLAY_N", default=10)
    
    # Initialize stop/threads tracking so we can start immediately if AUTO_START
    app.state.stop_event = threading.Event()
    app.state.threads = []

    AUTO_START = os.getenv("AUTO_START", "false").lower() == "true"

    # Precompute results file path for this run when exporting is enabled
    app.state.results_file = None
    if app.state.export_enabled:
        run_id = os.getenv("RUN_ID", "").strip()
        if run_id:
            app.state.results_file = os.path.join(
                app.state.results_dir,
                f"{app.state.participant_name}-run{run_id}.json"
            )
            logger.info(f"Preselected results file (RUN_ID={run_id}): {app.state.results_file}")
        else:
            app.state.results_file = _next_run_json(app.state.results_dir, app.state.participant_name)
            logger.info(f"Preselected results file (auto): {app.state.results_file}")

    # Start background roles
    if app.state.chain_display:
        chain_display_thread = threading.Thread(
            target=app.state.blockchain_manager.periodic_attestation_chain_display,
            args=(app.state.chain_display_n, app.state.chain_display_sec),
            name="ChainDisplay",
            daemon=True
        )
        chain_display_thread.start()
        app.state.threads.append(chain_display_thread)
    else:
        if AUTO_START:
            start_attestation(app)

@app.get("/")
def root_endpoint():
    return {"message": "SECaaS running."}

@app.post("/start")
def start_endpoint():
    start_attestation(app)
    return {"message": "Attestation started."}

@app.post("/stop")
def stop_endpoint():
    stop_attestation(app)
    return {"message": "Attestation stopped."}
    
@app.post("/reset")
def reset_attestation_chain():
    try:
        blockchain_manager = app.state.blockchain_manager
        tx_hash = blockchain_manager.reset_attestation_chain(wait=True, timeout=30)
        return {"message": "Chain reset", "tx_hash": tx_hash}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})
    
@app.on_event("shutdown")
async def shutdown():
    stop_attestation(app)
    getattr(app.state.blockchain_manager, "close", lambda: None)()