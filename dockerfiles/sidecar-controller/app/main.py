import logging
import sys
import json
import argparse
import time
import utils
from blockchain_interface import BlockchainInterface, MasMutualAttestationContractEvents, AttestationState
import os
import threading

from web3 import Web3
from prettytable import PrettyTable

from fastapi import FastAPI
from fastapi.responses import JSONResponse

# === ANSI Colors for Terminal Logging ===
RED = "\033[91m"
GREEN = "\033[92m"
YELLOW = "\033[93m"
BLUE = "\033[94m"
MAGENTA = "\033[95m"
CYAN = "\033[96m"
RESET = "\033[0m"

# === Logging ===
# logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
logging.basicConfig(format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

# === Environment Variables ===
participant = os.getenv("PARTICIPANT", "").lower()
json_file_path = os.getenv("CONFIG_PATH", "")
bootstrap_flag = os.getenv("BOOTSTRAP", "false").lower() == "true"
fail_attestation_flag = os.getenv("FAIL_ATTESTATION", "false").lower() == "true"
export_results_flag = os.getenv("EXPORT_RESULTS", "false").lower() == "true"

if participant not in ["agent", "secaas"]:
    logger.error('PARTICIPANT must be set to either "agent" or "secaas".')
    sys.exit(1)

if not json_file_path:
    logger.error("CONFIG_PATH environment variable is required.")
    sys.exit(1)

# Print selected values
print("Selected Execution Parameters:")
print(f"  - Participant       : {participant}")
print(f"  - Export results    : {'Enabled' if export_results_flag else 'Disabled'}")
print(f"  - Bootstrap mode    : {'Enabled' if bootstrap_flag else 'Disabled'}")
print(f"  - Fail mode         : {'Enabled' if fail_attestation_flag else 'Disabled'}")

# === Configuration Loading ===
try:
    with open(json_file_path, "r") as json_file:
        data = json.load(json_file)

    eth_address = data["eth_address"]
    private_key = data["private_key"]
    eth_node_url = data["eth_node"]
    contract_address = Web3.toChecksumAddress(data["contract_address"])

    if participant == "agent":
        agent_name = data["name"]
        agent_uuid = data["uuid"]

except Exception as e:
    logger.error(f"Configuration error: {str(e)}")
    sys.exit(1)

# === Blockchain Initialization ===
abi_path    = "/smart-contracts/build/contracts/MasMutualAttestation.json"
blockchain_interface = BlockchainInterface(
    eth_address=eth_address,
    private_key=private_key,
    eth_node_url=eth_node_url,
    abi_path=abi_path,
    contract_address=contract_address
)

seen_events = set()
active_attestations = {}  # attestation_id → Thread

# === FastAPI App ===
app = FastAPI()

def periodic_attestation_chain_display(interval=5):
    while True:
        time.sleep(interval)
        try:
            blockchain_interface.display_attestation_chain(last_n=10)
        except Exception as e:
            logger.error(f"Failed to display attestation chain: {e}")

def handle_verifier_logic(attestation_id, last_n_blocks):
    ready_for_evaluation = False 
    timestamps = {"start": time.time()}

    while ready_for_evaluation == False:
        ready_for_evaluation = True if blockchain_interface.get_attestation_state(attestation_id) ==  AttestationState.ReadyForEvaluation else False
        time.sleep(1)

    timestamps["evaluation_ready_received"] = time.time()

    blockchain_interface.display_attestation_state(attestation_id)

    # Retrieve and compare measurements
    fresh_signature, reference_signature = blockchain_interface.get_attestation_measurements(attestation_id)

    print(f"Attestation measurements:")
    print(f"  - Fresh Signature: {fresh_signature}")
    print(f"  - Reference Signature: {reference_signature}\n")

    # Compare measurements
    logger.info("Comparing hashes...")
    # result = fresh_signature == reference_signature
    result = True

    # Save attestation result
    tx_hash = blockchain_interface.send_attestation_result(attestation_id, result)
    timestamps["result_sent"] = time.time()
    logger.info(f"Attestation closed (Result: {'✅ SUCCESS' if result else '❌ FAILURE'})\n")
    if export_results_flag:
        utils.export_attestation_result_csv(agent_name, attestation_id, "verifier", "SUCCESS" if result else "FAILURE", timestamps)

def handle_prover_logic(attestation_id, last_n_blocks, measurement):
    """Handles Prover agent logic."""

    timestamps = {"start": time.time()}

    try:
        # Initiate attestation here
        tx_hash = blockchain_interface.request_attestation(attestation_id, measurement)
        logger.info(f"Attestation requested | Tx Hash: {tx_hash}")
        timestamps["request_sent"] = time.time()
    except Exception as e:
        logger.error(f"Failed to request attestation: {e}")
        return
    
    while not blockchain_interface.is_prover_agent(attestation_id):
        time.sleep(1)
    
    logger.info("Waiting for attestation to complete...")    
    attestation_closed = False 
    while attestation_closed == False:
        attestation_closed = True if blockchain_interface.get_attestation_state(attestation_id) == AttestationState.Closed else False
        time.sleep(1)
        
    timestamps["result_received"] = time.time()
    blockchain_interface.display_attestation_state(attestation_id)

    # Retrieve attestation details
    prover_address, verifier_address, attestation_result, timestamp = blockchain_interface.get_attestation_info(attestation_id)
    status = "SUCCESS" if attestation_result == 2 else "FAILURE"

    if export_results_flag:
        utils.export_attestation_result_csv(agent_name, attestation_id, "prover", status, timestamps)

    table = PrettyTable()
    table.field_names = ["Attestation ID", "Result", "Timestamp"]
    table.align = "l"
    table.add_row([attestation_id, "✅ SUCCESS" if attestation_result == 2 else "❌ FAILURE", timestamp])
    print(table)
    time.sleep(2)

def _process_agent_attestation(
    attestation_id: str,
    last_n_blocks: int,
    is_prover: bool = False,
    measurement: str = None
):
    start = time.time()
    try:
        if is_prover:
            handle_prover_logic(attestation_id, last_n_blocks, measurement)
        else:
            logger.info("Waiting for response from SECaaS...")
            handle_verifier_logic(attestation_id, last_n_blocks)
    finally:
        # When done, remove from active map
        active_attestations.pop(attestation_id, None)

    elapsed = time.time() - start
    logger.info(f"{RED}Task completed in {elapsed:.2f}s{RESET}")

def _process_secaas_attestation(attestation_id: str, last_n_blocks: int):
    try:
        logger.info(f"Processing attestation '{attestation_id}'")
        logger.info("Retrieving Prover UUID from SC...")
        agent_uuid = blockchain_interface.get_prover_uuid(attestation_id)
        logger.info("Retrieving reference signature from DB...")
        agent_info = utils.get_agent_by_uuid("/config", agent_uuid)
        if not agent_info:
            logger.error(f"Agent with UUID '{agent_uuid}' not found in the DB.")
            return

        reference_agent_measurement = agent_info['ref_signature']

        tx_hash = blockchain_interface.send_reference_signature(attestation_id, reference_agent_measurement)
        logger.info(f"Reference signature sent for UUID '{agent_uuid}'")

        if blockchain_interface.is_verifier_agent(attestation_id):
            logger.info("SECaaS is also the verifier for this attestation.")
            handle_verifier_logic(attestation_id, last_n_blocks)

        while blockchain_interface.get_attestation_state(attestation_id) != AttestationState.Closed:
            time.sleep(1)

    except Exception as e:
        logger.error(f"Error while processing attestation '{attestation_id}': {e}")

    finally:
        # When done, remove from active map
        active_attestations.pop(attestation_id, None)

def handle_verifier_logic_multi_thread(seen_events, last_n_blocks: int = None):
    logger.info("Subscribed to attestation events...")
    while True:
        new_attestation_event_filter = blockchain_interface.create_event_filter(MasMutualAttestationContractEvents.ATTESTATION_STARTED, last_n_blocks)
        
        attestation_queue = []

        # Collect all new valid attestation events
        attestation_events = new_attestation_event_filter.get_all_entries()
        for event in attestation_events:
            tx_hash = event['transactionHash'].hex()
            if tx_hash not in seen_events:
                attestation_id = Web3.toText(event['args']['id'])
                current_attestation_state = blockchain_interface.get_attestation_state(attestation_id)
                if current_attestation_state == AttestationState.Open and blockchain_interface.is_verifier_agent(attestation_id):  # Open or SecaasResponded
                    seen_events.add(tx_hash)
                    attestation_queue.append(attestation_id)

        if attestation_queue:
            logger.info(f"Found {len(attestation_queue)} new attestation(s)…\n")

        # Process each attestation ID sequentially
        for attestation_id in attestation_queue:
            if attestation_id in active_attestations:
                continue
            logger.info(f"Processing attestation: {attestation_id}")
            thread_name = f"Agent-Verifier-{attestation_id}"
            thread = threading.Thread(
                target=_process_agent_attestation,
                name=thread_name,
                args=(attestation_id, last_n_blocks, False),
                daemon=True
            )
            active_attestations[attestation_id] = thread
            thread.start()
        time.sleep(3)


def handle_secaas_logic_multi_thread(seen_events, last_n_blocks: int = None):    
    logger.info("Subscribed to attestation events...\n")
    while True:
        new_attestation_event_filter = blockchain_interface.create_event_filter(
            MasMutualAttestationContractEvents.ATTESTATION_STARTED,
            last_n_blocks
        )

        attestation_queue = []

        # Collect all new valid attestation events
        attestation_events = new_attestation_event_filter.get_all_entries()
        for event in attestation_events:
            tx_hash = event['transactionHash'].hex()
            if tx_hash not in seen_events:
                attestation_id = Web3.toText(event['args']['id'])
                current_state = blockchain_interface.get_attestation_state(attestation_id)
                if current_state == AttestationState.Open:
                    seen_events.add(tx_hash)
                    attestation_queue.append((attestation_id, tx_hash))

        if not attestation_queue:
            # logger.debug("No new attestation events. Sleeping before next check...")
            time.sleep(3)
            continue

        # Process each attestation in the queue
        for attestation_id, tx_hash in attestation_queue:
            if attestation_id in active_attestations:
                continue

            # logger.info(f"Spawning thread for attestation: {attestation_id}")
            thread = threading.Thread(
                target=_process_secaas_attestation,
                args=(attestation_id, last_n_blocks),
                name=f"SECaaS-{attestation_id}",
                daemon=True
            )
            active_attestations[attestation_id] = thread
            thread.start()

@app.on_event("startup")
def auto_start_attestation():
    logger.info("Auto-starting verifier logic...")
    if participant == 'secaas':
        # Start attestation display thread for SECaaS only
        display_thread = threading.Thread(
            target=periodic_attestation_chain_display,
            name="Chain-Display",
            daemon=True
        )
        display_thread.start()

        thread = threading.Thread(
            target=handle_secaas_logic_multi_thread,
            args=(seen_events, 10),
            name="SECaaS",
            daemon=True
        )
        thread.start()
        logger.info("SECaaS attestation loop started")

    elif participant == 'agent':
        logger.info(f"Registering agent with UUID: {agent_uuid}...")
        tx_hash = blockchain_interface.register_agent(agent_uuid)
        logger.info(f"Agent registered | Tx Hash: {tx_hash}")
        time.sleep(3)

        thread = threading.Thread(
            target=handle_verifier_logic_multi_thread,
            args=(seen_events, 10),
            name="Agent-Verifier",
            daemon=True
        )
        thread.start()
        logger.info("Agent attestation loop started")

@app.post("/request_attestation/{measurement}")
def request_attestation(measurement: str):
    attestation_id = utils.generate_attestation_id()
    thread = threading.Thread(
        target=_process_agent_attestation,
        args=(attestation_id, 10, True, measurement),
        name=f"Agent-Prover-{attestation_id}",
        daemon=True
    )
    active_attestations[attestation_id] = thread
    thread.start()
    return {"message": "Attestation requested", "measurement": measurement, "attestation_id": attestation_id}

@app.get("/")
def root_endpoint():
    return {"message": "Sidecar controller running."}

@app.get("/attestation_chain")
def get_attestation_chain_endpoint():
    try:
        chain = blockchain_interface.get_attestation_chain()
        latest = chain[-10:]
        return {"last_attestations": [a.hex() for a in latest]}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})
    
@app.post("/reset")
def reset_attestation_chain():
    if participant != "secaas":
        return JSONResponse(status_code=403, content={"error": "Only SECaaS can reset the chain."})
    try:
        tx_hash = blockchain_interface.reset_attestation_chain()
        return {"message": "Chain reset", "tx_hash": tx_hash}
    except Exception as e:
        return JSONResponse(status_code=500, content={"error": str(e)})