import logging
import json
import utils
from blockchain_client import BlockchainClient

import os
import threading

from web3 import Web3
from fastapi import FastAPI
from fastapi.responses import JSONResponse

from prover import run_prover_logic_continuous_mode
from verifier import run_verifier_logic_sequential
from secaas import run_secaas_logic
from RedisStorageBackend import RedisStorageBackend
from MemoryStorageBackend import MemoryStorageBackend

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

def start_attestation(app: FastAPI):
    # Create stop event / threads list if not present (idempotent)
    if not hasattr(app.state, "stop_event"):
        app.state.stop_event = threading.Event()
    if not hasattr(app.state, "threads"):
        app.state.threads = []

    # If we’re restarting, make sure stop is cleared
    app.state.stop_event.clear()

    if app.state.export_enabled:
        utils.ensure_results_initialized(app.state.results_dir, app.state.participant_name)

    if app.state.participant == "secaas":
        logger.info("SECaaS attestation loop started")
        t = threading.Thread(target=run_secaas_logic, args=(app, app.state.stop_event), name="SECaaS", daemon=True)
        t.start()
        app.state.threads.append(t)
    elif app.state.participant == "agent":
        if app.state.measuring:
            logger.info("Agent attestation already running; ignoring start request")
            return
        app.state.measuring = True

        logger.info("Agent attestation loop started")
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
        t1.start(); t2.start()
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
            logger.warning(f"Failed joining thread {t.name}: {e}")

    # Reset the thread list after stopping
    if hasattr(app.state, "threads"):
        app.state.threads.clear()

    if app.state.export_enabled:
        utils.mark_experiment_stop(app.state.results_dir, app.state.participant_name)

@app.on_event("startup")
async def startup():

    # Read env & config
    PARTICIPANT = utils.get_env_str("PARTICIPANT", default="agent")
    if PARTICIPANT not in ["agent", "secaas"]:
        logger.error('PARTICIPANT must be set to either "agent" or "secaas".')
        raise SystemExit(1)
    
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
        if 'cmd_name' in data:
            cmd_name = data['cmd_name']
            app.state.cmd_name = cmd_name

        if 'text_section_size' in data:
            text_section_size = int(data['text_section_size'])
            app.state.text_section_size = text_section_size

        if 'offset' in data:
            offset = int(data['offset'])
            app.state.offset = offset
    except Exception as e:
        logger.error(f"Configuration error: {str(e)}")
        raise SystemExit(1)

    # App state
    app.state.participant = PARTICIPANT
    app.state.blockchain_client = BlockchainClient(
        eth_address=eth_address,
        private_key=private_key,
        eth_node_url=eth_node_url,
        abi_path="/smart-contracts/artifacts/contracts/MasMutualAttestation.sol/MasMutualAttestation.json",
        contract_address=contract_address
    )
    app.state.active_attestations = {}   # attestation_id → Thread
    app.state.export_enabled = os.getenv("EXPORT_RESULTS", "false").lower() == "true"
    app.state.participant_name = (data["name"] if PARTICIPANT == "agent" else "secaas")
    app.state.results_dir = utils.get_env_str("RESULTS_DIR", default="/experiments/data/attestation-times")
    app.state.fail_attestation_flag = os.getenv("FAIL_ATTESTATION", "false").lower() == "true"
    app.state.event_confirmations = utils.get_env_int("EVENT_CONFIRMATIONS", default=2)
    app.state.event_batch_size = utils.get_env_int("EVENT_BATCH_SIZE", default=2000)
    app.state.event_poll_interval = utils.get_env_float("EVENT_POLL_INTERVAL", default=0.5)
    app.state.event_lookback_blocks = utils.get_env_int("EVENT_LOOKBACK_BLOCKS", default=0)
    app.state.event_checkpoint_dir = utils.get_env_str("EVENT_CHECKPOINT_DIR", default="/checkpoints")
    app.state.chain_display = utils.get_env_str("CHAIN_DISPLAY", default="false").strip().lower() in ("true", "yes", "1")
    app.state.chain_display_sec = utils.get_env_int("CHAIN_DISPLAY_SEC", default=30)
    app.state.chain_display_n   = utils.get_env_int("CHAIN_DISPLAY_N", default=10)
    
    # Initialize stop/threads tracking so we can start immediately if AUTO_START
    app.state.stop_event = threading.Event()
    app.state.threads = []

    USE_REDIS = os.getenv("USE_REDIS", "false").lower() == "true"
    app.state.storage = RedisStorageBackend() if USE_REDIS else MemoryStorageBackend()
    app.state.prover_threshold = utils.get_env_int("PROVER_THRESHOLD", default=300)
    app.state.memory_storage_file = utils.get_env_str("MEMORY_STORAGE_FILE", default="")
    app.state.measuring = False
    app.state.digests = []
    app.state.digests_count = 0
    app.state.final_digest = ""
    AUTO_START = os.getenv("AUTO_START", "false").lower() == "true"

    # Start background roles
    if app.state.chain_display:
        chain_display_thread = threading.Thread(
            target=app.state.blockchain_client.periodic_attestation_chain_display,
            args=(app.state.chain_display_n, app.state.chain_display_sec),
            name="ChainDisplay",
            daemon=True
        )
        chain_display_thread.start()
        app.state.threads.append(chain_display_thread)
    else:
        if PARTICIPANT == "agent" and not app.state.blockchain_client.is_registered():
            logger.info(f"Registering agent '{app.state.participant_name}' ({eth_address}) ...")
            tx_hash = app.state.blockchain_client.register_agent(app.state.participant_name, wait=True, timeout=30)
            logger.info(f"{app.state.participant_name} registered (Tx: {tx_hash})")
        if AUTO_START:
            start_attestation(app)

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
    
@app.post("/reset")
def reset_attestation_chain():
    if app.state.participant != "secaas":
        return JSONResponse(status_code=403, content={"error": "Only SECaaS can reset the chain."})
    try:
        blockchain_client = app.state.blockchain_client
        tx_hash = blockchain_client.reset_attestation_chain()
        return {"message": "Chain reset", "tx_hash": tx_hash}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})
    
@app.on_event("shutdown")
async def shutdown():
    stop_attestation(app)

    # graceful cleanup
    getattr(app.state.blockchain_client, "close", lambda: None)()