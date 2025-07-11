import logging
import sys
import json
import argparse
import time
import utils
import blockchain_interface
# import blockchain_interface_multi_thread as blockchain_interface
import os
import threading

from web3 import Web3
from prettytable import PrettyTable


# Initialize logging
# logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
logging.basicConfig(format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

# Read required environment variables
participant = os.getenv("PARTICIPANT", "").lower()
json_file_path = os.getenv("CONFIG_PATH", "")
reset_chain_flag = os.getenv("RESET_CHAIN", "false").lower() == "true"
bootstrap_flag = os.getenv("BOOTSTRAP", "false").lower() == "true"
remove_agent_flag = os.getenv("REMOVE_AGENT", "false").lower() == "true"
fail_attestation_flag = os.getenv("FAIL_ATTESTATION", "false").lower() == "true"
export_results_flag = os.getenv("EXPORT_RESULTS", "false").lower() == "true"

# Validate required env vars
if participant not in ["agent", "secaas"]:
    logger.error('PARTICIPANT must be set to either "agent" or "secaas".')
    sys.exit(1)

if not json_file_path:
    logger.error("CONFIG_PATH environment variable is required.")
    sys.exit(1)

# Validate flag usage based on participant role
if participant == "secaas" and (bootstrap_flag or remove_agent_flag or fail_attestation_flag):
    logger.error("Only agents can use BOOTSTRAP, REMOVE_AGENT, or FAIL_ATTESTATION.")
    sys.exit(1)
elif participant == "agent" and reset_chain_flag:
    logger.error("Only SECaaS can use RESET_CHAIN.")
    sys.exit(1)

# Print selected values
print("Selected Execution Parameters:")
print(f"  - Participant       : {participant}")
print(f"  - Export results    : {'Enabled' if export_results_flag else 'Disabled'}")
print(f"  - Bootstrap mode    : {'Enabled' if bootstrap_flag else 'Disabled'}")
print(f"  - Fail mode         : {'Enabled' if fail_attestation_flag else 'Disabled'}")

# Load configuration
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
        reference_measurement = data["ref_signature"]

except FileNotFoundError:
    logger.error(f"Configuration file not found: {json_file_path}")
    sys.exit(1)
except KeyError as e:
    logger.error(f"Missing key {e} in configuration file")
    sys.exit(1)
except Exception as e:
    logger.error(f"Unexpected error: {str(e)}")
    sys.exit(1)


# Initialize Web3 + contract via blockchain_interface
abi_path    = "/smart-contracts/build/contracts/MasMutualAttestation.json"
mas_contract= blockchain_interface.init_web3_interface(
    _eth_address    = eth_address,
    _private_key    = private_key,
    eth_node_url    = eth_node_url,
    abi_path        = abi_path,
    contract_address= contract_address
)
seen_events = set()
active_attestations = {}  # attestation_id → Thread

def handle_verifier_logic(attestation_id, eth_address, contract, seen_events, last_n_blocks):
    """Handles Verifier agent logic."""
    ready_for_evaluation = False 
    timestamps = {"start": time.time()}

    while ready_for_evaluation == False:
        ready_for_evaluation = True if blockchain_interface.get_attestation_state(attestation_id, contract) == 3 else False
        time.sleep(1)

    timestamps["evaluation_ready_received"] = time.time()

    blockchain_interface.display_attestation_state(attestation_id, contract)

    # Retrieve and compare measurements
    fresh_signature, reference_signature = blockchain_interface.get_attestation_measurements(attestation_id, eth_address, contract)

    print(f"Attestation measurements:")
    print(f"  - Fresh Signature: {fresh_signature}")
    print(f"  - Reference Signature: {reference_signature}\n")

    # Compare measurements
    logger.info("Comparing hashes...")
    result = fresh_signature == reference_signature

    # Save attestation result
    tx_hash = blockchain_interface.send_attestation_result(attestation_id, result, eth_address, contract)
    timestamps["result_sent"] = time.time()
    logger.info(f"Attestation closed (Result: {'✅ SUCCESS' if result else '❌ FAILURE'})\n")
    if export_results_flag:
        utils.export_attestation_result_csv(agent_name, attestation_id, "verifier", "SUCCESS" if result else "FAILURE", timestamps)

def handle_prover_logic(attestation_id, eth_address, contract, seen_events, agent_uuid, last_n_blocks, fail_attestation_flag):
    """Handles Prover agent logic."""

    timestamps = {"start": time.time()}

    # logger.info("Generating fresh signature...")

    # Modify to use the runtime fresh signature generated by the sidecar-measurer
    agent_info = utils.get_agent_by_uuid("/config", agent_uuid)    
    fresh_signature = agent_info['ref_signature']

    # If the fail flag is enabled, modify the signature to an incorrect value
    if fail_attestation_flag:
        logger.warning("Fail Attestation flag is set. Modifying the fresh signature to trigger failure.")
        fresh_signature = "".join("F" if c.isalpha() else c for c in fresh_signature)

    timestamps["signature_prepared"] = time.time()
    
    tx_hash = blockchain_interface.send_fresh_signature(attestation_id, fresh_signature, eth_address, contract)
    timestamps["signature_sent"] = time.time()
    logger.info(f"Fresh signature sent: {fresh_signature}\n")

    logger.info("Waiting for attestation to complete...")    
    attestation_closed = False 
    while attestation_closed == False:
        attestation_closed = True if blockchain_interface.get_attestation_state(attestation_id, contract) == 4 else False
        time.sleep(1)
        
    timestamps["result_received"] = time.time()
    blockchain_interface.display_attestation_state(attestation_id, contract)

    # Retrieve attestation details
    prover_address, verifier_address, attestation_result, timestamp = blockchain_interface.get_attestation_info(attestation_id, contract)
    status = "SUCCESS" if attestation_result == 2 else "FAILURE"

    if export_results_flag:
        utils.export_attestation_result_csv(agent_name, attestation_id, "prover", status, timestamps)

    table = PrettyTable()
    table.field_names = ["Attestation ID", "Result", "Timestamp"]
    table.align = "l"
    table.add_row([attestation_id, "✅ SUCCESS" if attestation_result == 2 else "❌ FAILURE", timestamp])
    print(table)
    time.sleep(3)
    
def _process_attestation(  # helper to wrap prover/verifier logic
    attestation_id, eth_address, contract, seen_events,
    agent_uuid, fail_attestation_flag, last_n_blocks
):
    start = time.time()
    role = None
    try:
        # Decide role
        if blockchain_interface.is_verifier_agent(attestation_id, eth_address, contract):
            role = "Verifier"
            logger.info("You are the Verifier agent! - starting work\n")
            logger.info("Waiting for responses from SECaaS and Prover...")
            handle_verifier_logic(attestation_id, eth_address, contract, seen_events, last_n_blocks)
        elif blockchain_interface.is_prover_agent(attestation_id, eth_address, contract):
            role = "Prover"
            logger.info("You are the Prover agent! - starting work\n")
            handle_prover_logic(attestation_id, eth_address, contract, seen_events, agent_uuid, last_n_blocks, fail_attestation_flag)
        else:
            logger.warning("You are not involved in this attestation. Ignoring...\n") 
            time.sleep(3)
    finally:
        # When done, remove from active map
        active_attestations.pop(attestation_id, None)
    if role in ("Verifier", "Prover"):
        elapsed = time.time() - start
        logger.info(f"{role} duration: {elapsed:.2f}s")

def handle_agent_logic_multi_thread(eth_address, contract, seen_events, agent_uuid, fail_attestation_flag: bool = False, last_n_blocks: int = None):
    """Handles Agent registration and attestation process with support for multiple attestation events."""
    logger.info("Subscribed to attestation events...")
    while True:
        new_attestation_event_filter = blockchain_interface.create_event_filter(contract, blockchain_interface.MasMutualAttestationContractEvents.ATTESTATION_STARTED, last_n_blocks)
        
        attestation_queue = []

        # Collect all new valid attestation events
        attestation_events = new_attestation_event_filter.get_all_entries()
        for event in attestation_events:
            tx_hash = event['transactionHash'].hex()
            if tx_hash not in seen_events:
                attestation_id = event["args"]["id"].hex()

                current_attestation_state = blockchain_interface.get_attestation_state(attestation_id, contract)
                if current_attestation_state in [0, 1]:  # Open or SecaasResponded
                    seen_events.add(tx_hash)
                    attestation_queue.append(attestation_id)

        if attestation_queue:
            logger.info(f"Found {len(attestation_queue)} new attestation(s)…\n")

        # Process each attestation ID sequentially
        for attestation_id in attestation_queue:
            if attestation_id in active_attestations:
                continue
            
            logger.info(f"Processing attestation: {attestation_id}")
            short_id = attestation_id[:4]
            thread_name = f"Attest-{short_id}"

            t = threading.Thread(
                target=_process_attestation,
                name=thread_name,
                args=(
                    attestation_id, eth_address, contract, seen_events,
                    agent_uuid, fail_attestation_flag, last_n_blocks
                ),
                daemon=True
            )
            active_attestations[attestation_id] = t
            t.start()
        # nothing to block on here — immediately loop again
        time.sleep(1)

def handle_agent_logic(eth_address, contract, seen_events, agent_uuid, fail_attestation_flag: bool = False, last_n_blocks: int = None):
    """Handles Agent registration and attestation process with support for multiple attestation events."""
    logger.info("Subscribed to attestation events...")
    while True:
        new_attestation_event_filter = blockchain_interface.create_event_filter(contract, blockchain_interface.MasMutualAttestationContractEvents.ATTESTATION_STARTED, last_n_blocks)
        
        attestation_queue = []

        # Collect all new valid attestation events
        attestation_events = new_attestation_event_filter.get_all_entries()
        for event in attestation_events:
            tx_hash = event['transactionHash'].hex()
            if tx_hash not in seen_events:
                attestation_id = event["args"]["id"].hex()

                current_attestation_state = blockchain_interface.get_attestation_state(attestation_id, contract)
                if current_attestation_state in [0, 1]:  # Open or SecaasResponded
                    seen_events.add(tx_hash)
                    attestation_queue.append((attestation_id, tx_hash))

        if attestation_queue:
            logger.info(f"Found {len(attestation_queue)} new attestation(s)…\n")

        if not attestation_queue:
            logger.debug("No new attestation events found. Sleeping before next poll...")
            time.sleep(3)
            continue

        # Process each attestation ID sequentially
        for attestation_id, tx_hash in attestation_queue:
            logger.info(f"Processing attestation: {attestation_id}")

            # blockchain_interface.display_attestation_state(attestation_id, contract)

            is_verifier = blockchain_interface.is_verifier_agent(attestation_id, eth_address, contract)
            is_prover = blockchain_interface.is_prover_agent(attestation_id, eth_address, contract)

            start_time = time.time()

            if is_verifier:
                logger.info("You are the Verifier agent!\n")
                logger.info("Waiting for responses from SECaaS and Prover...")
                handle_verifier_logic(attestation_id, eth_address, contract, seen_events, last_n_blocks)
                duration = time.time() - start_time
                logger.info(f"Attestation duration (Verifier perspective): {duration:.2f} seconds\n")

            elif is_prover:
                logger.info("You are the Prover agent!\n")
                handle_prover_logic(attestation_id, eth_address, contract, seen_events, agent_uuid, last_n_blocks, fail_attestation_flag)
                duration = time.time() - start_time
                logger.info(f"Attestation duration (Prover perspective): {duration:.2f} seconds\n")

            else:
                logger.warning("You are not involved in this attestation. Ignoring...\n") 
                time.sleep(3)
                continue

def handle_secaas_logic(eth_address, contract, seen_events, last_n_blocks: int = None):
    """Handles SECaaS logic with support for multiple attestation events."""
    
    blockchain_interface.display_attestation_chain(contract, last_n=20)
    
    logger.info("Subscribed to attestation events...\n")

    while True:
        new_attestation_event_filter = blockchain_interface.create_event_filter(
            contract,
            blockchain_interface.MasMutualAttestationContractEvents.ATTESTATION_STARTED,
            last_n_blocks
        )

        attestation_queue = []

        # Collect all new valid attestation events
        attestation_events = new_attestation_event_filter.get_all_entries()
        for event in attestation_events:
            tx_hash = event['transactionHash'].hex()
            if tx_hash not in seen_events:
                attestation_id = event["args"]["id"].hex()
                current_state = blockchain_interface.get_attestation_state(attestation_id, contract)
                if current_state in [0, 2]:  # Open or ProverResponded
                    seen_events.add(tx_hash)
                    attestation_queue.append((attestation_id, tx_hash))

        if not attestation_queue:
            # logger.debug("No new attestation events. Sleeping before next check...")
            time.sleep(3)
            continue

        # Process each attestation in the queue
        for attestation_id, tx_hash in attestation_queue:
            logger.info(f"Processing attestation '{attestation_id}'\n")

            try:
                logger.info(f"Retrieving Prover UUID from SC...")
                agent_uuid = blockchain_interface.get_prover_uuid(attestation_id, eth_address, contract)
                
                logger.info(f"Retrieving reference signature from DB...")

                # Modify to retrieve the UUID and reference signature from the SECaaS DB

                agent_info = utils.get_agent_by_uuid("/config", agent_uuid)
                if not agent_info:
                    logger.error(f"Agent with UUID '{agent_uuid}' not found in the DB.")
                    continue

                reference_agent_measurement = agent_info['ref_signature']

                tx_hash = blockchain_interface.send_reference_signature(attestation_id, reference_agent_measurement, eth_address, contract)
                logger.info(f"Reference signature for agent with UUID '{agent_uuid}' sent\n")

                # If SECaaS is also verifier, process result
                if blockchain_interface.is_verifier_agent(attestation_id, eth_address, contract):
                    logger.info("SECaaS is also the verifier\n")
                    handle_verifier_logic(attestation_id, eth_address, contract, seen_events, last_n_blocks)
                    time.sleep(3)

                # Wait for attestation closure
                while blockchain_interface.get_attestation_state(attestation_id, contract) != 4:
                    time.sleep(1)

                blockchain_interface.display_attestation_chain(contract)

            except Exception as e:
                logger.error(f"Error while processing attestation '{attestation_id}': {e}")

if __name__ == "__main__":
    try:
        if participant == 'secaas':
            if reset_chain_flag:
                logger.info(f"Reset Chain       : {'Enabled' if reset_chain_flag else 'Disabled'}")
                logger.info("Resetting attestation chain...")
                tx_hash = blockchain_interface.reset_attestation_chain(eth_address, mas_contract)
                logger.info(f"Attestation chain reset | Tx Hash: {tx_hash}")
                sys.exit(1)

            handle_secaas_logic(eth_address, mas_contract, seen_events, last_n_blocks=10)

        elif participant == 'agent':
            if bootstrap_flag:
                print()
                # logger.info("Registering agent...")
                # tx_hash = blockchain_interface.register_agent(agent_uuid, eth_address, mas_contract)
                # logger.info(f"Agent registered successfully | Tx: {tx_hash}")
        
            elif remove_agent_flag:
                logger.info(f"Remove Agent      : {'Enabled' if remove_agent_flag else 'Disabled'}")
                logger.info("Removing agent...")
                tx_hash = blockchain_interface.remove_agent(eth_address, mas_contract)
                logger.info(f"Agent removed successfully | Tx: {tx_hash}")
                sys.exit(1)

            # handle_agent_logic(eth_address, mas_contract, seen_events, agent_uuid, fail_attestation_flag, last_n_blocks=10)
            handle_agent_logic_multi_thread(eth_address, mas_contract, seen_events, agent_uuid, fail_attestation_flag, last_n_blocks=10)

    except Exception as e:
            print(f"An error occurred in the main execution: {str(e)}")
            sys.exit(1)





