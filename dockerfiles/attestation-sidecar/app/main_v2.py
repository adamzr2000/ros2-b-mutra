import logging
import json
import utils
from blockchain_client import BlockchainClient
from queue import Queue

import os
import threading

from web3 import Web3
from fastapi import FastAPI
from fastapi.responses import JSONResponse
from pydantic import BaseModel

from prover import run_prover_and_cleanup, handle_prover_logic_continuous_mode
from verifier import handle_verifier_logic_sequential
from secaas import handle_secaas_logic
from RedisStorageBackend import RedisStorageBackend
from MemoryStorageBackend import MemoryStorageBackend

# === ANSI + Logging (same as you have) ===
GREEN = "\033[92m"; YELLOW = "\033[93m"; BLUE = "\033[94m"; MAGENTA = "\033[95m"; CYAN = "\033[96m"; RESET = "\033[0m"

# === Logging ===
LOG_LEVEL = os.getenv("LOG_LEVEL", "INFO").upper()

if LOG_LEVEL == "NONE":
    logging.disable(logging.CRITICAL)  # silence everything
else:
    numeric_level = getattr(logging, LOG_LEVEL, logging.INFO)
    logging.basicConfig(format="%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s", level=numeric_level)

app = FastAPI()
logger = logging.getLogger(__name__)

def start_attestation(app: FastAPI):
    if app.state.export_enabled:
        utils.ensure_results_initialized(app.state.results_dir, app.state.participant_name)

    if app.state.participant == "secaas":
        secaas_thread = threading.Thread(target=handle_secaas_logic, args=(app,), name="SECaaS", daemon=True)
        secaas_thread.start()
        logger.info("SECaaS attestation loop started")
    elif app.state.participant == "agent":
        agent_prover_thread = threading.Thread(target=handle_prover_logic_continuous_mode, 
                            args=(app,), name=f"off-chain-prover", daemon=True)
        agent_prover_thread.start()
        agent_verifier_thread = threading.Thread(target=handle_verifier_logic_sequential, 
                            args=(app,), name=app.state.participant_name, daemon=True)
        agent_verifier_thread.start()
        logger.info("Agent attestation loop started")

@app.on_event("startup")
async def startup():

    # Read env & config
    PARTICIPANT = utils.get_env_str("PARTICIPANT", default="")
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
        cmd_name = data['cmd_name']
        text_section_size = data['text_section_size'] 
        offset = data['offset'] 
        cmd_name = data['cmd_name']         
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
    
    prover_config  = {
        'cmd_name': cmd_name,
        'text_section_size': text_section_size,
        'offset': offset,
    }
    app.state.prover_config = prover_config
    USE_REDIS = os.getenv("USE_REDIS", "false").lower() == "true"
    app.state.storage = RedisStorageBackend() if USE_REDIS else MemoryStorageBackend()
    app.state.prover_thresold = utils.get_env_int("PROVER_THRESHOLD", default=300)
    app.state.memory_storage_file = utils.get_env_str("MEMORY_STORAGE_FILE", default="")
    app.state.measuring = False
    app.state.digests = []
    app.state.digests_count = 0
    app.state.final_digest = ""
    AUTO_START = os.getenv("AUTO_START", "false").lower() == "true"

    # Start background roles
    if app.state.chain_display:
        thread = threading.Thread(
            target=app.state.blockchain_client.periodic_attestation_chain_display,
            args=(app.state.chain_display_n, app.state.chain_display_sec),
            name="ChainDisplay",
            daemon=True
        )
        thread.start()
    else:
        if PARTICIPANT == "agent":
            logger.info(f"Registering agent '{app.state.participant_name}' ({eth_address}) ...")
            tx_hash = app.state.blockchain_client.register_agent(app.state.participant_name, wait=True, timeout=30)
            logger.info(f"{app.state.participant_name} registered (Tx: {tx_hash})")
        if AUTO_START:
            start_attestation()

# ==== Routes ====
class FreshSignatures(BaseModel):
    robot_signature: str
    prover_signature: str
    verifier_signature: str

@app.post("/send_evidence")
def send_evidence(payload: FreshSignatures):
    attestation_id = utils.generate_attestation_id()
    signatures = [payload.robot_signature, payload.prover_signature, payload.verifier_signature]
    thread = threading.Thread(
        target=run_prover_and_cleanup,
        args=(app, attestation_id, signatures),
        name=app.state.participant_name,
        daemon=True
    )
    app.state.active_attestations[attestation_id] = thread
    thread.start()
    return {"message": "Attestation started", "signatures": signatures, "attestation_id": attestation_id}

@app.get("/")
def root_endpoint():
    return {"message": "Attestation sidecar running."}
    
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
    
class StopAck(BaseModel):
    participant: str
    t_start: int
    t_end: int
    prover_count: int
    verifier_count: int
    oracle_count: int

@app.post("/data_collection/stop", response_model=StopAck)
def stop_experiment():
    utils.mark_experiment_stop(app.state.results_dir, app.state.participant_name)
    app.state.export_enabled = False
    logger.info(f"Data collection disabled for '{app.state.participant_name}'")
    path = os.path.join(app.state.results_dir, f"{app.state.participant_name}.json")
    with open(path, "r") as f:
        data = json.load(f)
    return StopAck(
        participant=app.state.participant_name,
        t_start=data.get("t_start"),
        t_end=data.get("t_end"),
        prover_count=len(data.get("prover", [])),
        verifier_count=len(data.get("verifier", [])),
        oracle_count=len(data.get("oracle", [])),
    )

@app.on_event("shutdown")
async def shutdown():
    if app.state.export_enabled:
        utils.mark_experiment_stop(app.state.results_dir, app.state.participant_name)

    # graceful cleanup
    getattr(app.state.blockchain_client, "close", lambda: None)()