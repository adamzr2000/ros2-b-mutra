import logging
import sys
import time
import json
import uuid
from web3 import Web3, WebsocketProvider
from web3.middleware import geth_poa_middleware
from dotenv import load_dotenv, dotenv_values
from enum import Enum
import utils as utils
import sys
import argparse

# Initialize logging
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

class MasMutualAttestationContractEvents(str, Enum):
    AGENT_REGISTERED = "AgentRegistered"
    AGENT_REMOVED = "AgentRemoved"
    ATTESTATION_STARTED = "AttestationStarted"
    SECAAS_RESPONDED = "SecaasResponded"
    PROVER_RESPONDED = "ProverResponded"
    READY_FOR_EVALUATION = "ReadyForEvaluation"
    ATTESTATION_COMPLETED = "AttestationCompleted"

# Parse participant role
parser = argparse.ArgumentParser(description='Initialize participant')
parser.add_argument('--participant', type=str, default='secaas|agent',
                         help='Flags: "secaas" for the SECaaS, "agent" for Agents')

args = parser.parse_args()
participant = args.participant

# Load configuration
json_file_path = "/home/agent/config/Secaas.json" if participant == "secaas" else utils.find_agent_json(directory="/home/agent/config")

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


# Initialize Web3 and Contract
try:
    web3 = Web3(WebsocketProvider(eth_node_url))
    web3.middleware_onion.inject(geth_poa_middleware, layer=0)

    if not web3.isConnected():
        raise ConnectionError("Failed to connect to Ethereum node")

    logger.info(f"Connected to Ethereum node {eth_node_url} | Geth Version: {web3.clientVersion}")
    
    with open("/home/agent/smart-contracts/build/contracts/MasMutualAttestation.json", "r") as abi_file:
            contract_abi = json.load(abi_file).get("abi", None)

    if not contract_abi:
        raise ValueError("ABI not found in contract JSON file")
    
    mas_contract = web3.eth.contract(address=contract_address, abi=contract_abi)

    # Number used to ensure the order of transactions (and prevent transaction replay attacks)
    nonce = web3.eth.getTransactionCount(eth_address)
    seen_events = set()

except Exception as e:
    logger.error(f"Web3 initialization failed: {str(e)}")
    sys.exit(1)
    
def send_signed_transaction(build_transaction):
    """
    Sends a signed transaction to the blockchain network using the private key.
    
    Args:
        build_transaction (dict): The transaction data to be sent.
    
    Returns:
        str: The transaction hash of the sent transaction.
    """
    global nonce
    # Sign the transaction
    signed_txn = web3.eth.account.signTransaction(build_transaction, private_key)
    # Send the signed transaction
    tx_hash = web3.eth.sendRawTransaction(signed_txn.rawTransaction)
    # Increment the nonce
    nonce += 1
    return tx_hash.hex()

def get_transaction_receipt(tx_hash: str) -> dict:
    """
    Retrieves details of the transaction receipt for the specified hash, including:
    
    - Block info: Block hash, block number, and timestamp.
    - Gas usage: Gas used and cumulative gas.
    - Status: Transaction success (1) or failure (0).
    - Sender/Receiver: from_address and to_address.
    - Logs: Event logs generated during the transaction.
    - Gas price: Actual gas price paid.
    - Timestamp: The timestamp of the block in which the transaction was included.
    
    Args:
        tx_hash (str): The transaction hash to retrieve the receipt.

    Returns:
        dict: A dictionary containing transaction receipt details and block timestamp, or an error message.
    """
    try:
        # Get the transaction receipt
        receipt = web3.eth.get_transaction_receipt(tx_hash)

        if receipt:
            # Convert HexBytes to strings for JSON serialization
            receipt_dict = dict(receipt)
            receipt_dict['blockHash'] = receipt_dict['blockHash'].hex()
            receipt_dict['transactionHash'] = receipt_dict['transactionHash'].hex()
            receipt_dict['logsBloom'] = receipt_dict['logsBloom'].hex()
            receipt_dict['logs'] = [dict(log) for log in receipt_dict['logs']]

            # Rename fields to be more descriptive
            receipt_dict['from_address'] = receipt_dict.pop('from')
            receipt_dict['to_address'] = receipt_dict.pop('to')

            # Convert nested hex values in logs
            for log in receipt_dict['logs']:
                log['blockHash'] = log['blockHash'].hex()
                log['transactionHash'] = log['transactionHash'].hex()
                log['topics'] = [topic.hex() for topic in log['topics']]

            # Retrieve the block number from the receipt
            block_number = receipt['blockNumber']

            # Fetch the block details using the block number
            block = web3.eth.get_block(block_number)

            # Add the block timestamp to the receipt dictionary
            receipt_dict['timestamp'] = block['timestamp']

            return receipt_dict

        else:
            raise Exception("Error: Transaction receipt not found")

    except Exception as e:
        raise Exception(f"An exception occurred: {str(e)}")

    
def create_event_filter(contract, event_name: MasMutualAttestationContractEvents, last_n_blocks: int = None):
    """
    Creates a filter to catch the specified event emitted by the smart contract.
    This function can be used to monitor events in real-time or from a certain number of past blocks.

    Args:
        contract: The contract instance to monitor events from.
        event_name (MasMutualAttestationContractEvents): The name of the smart contract event to create a filter for.
        last_n_blocks (int, optional): If provided, specifies the number of blocks to look back from the latest block.
                                       If not provided, it listens from the latest block onward.

    Returns:
        Filter: A filter for catching the specified event.
    """
    try:
        block = web3.eth.getBlock('latest')
        block_number = block['number']
        
        # If last_n_blocks is provided, look back, otherwise start from the latest block
        from_block = max(0, block_number - last_n_blocks) if last_n_blocks else block_number
        
        # Use the contract instance passed as an argument to access the events
        event_filter = getattr(contract.events, event_name.value).createFilter(fromBlock=web3.toHex(from_block))
        return event_filter
    except AttributeError:
        raise ValueError(f"Event '{event_name}' does not exist in the contract.")
    except Exception as e:
        raise Exception(f"An error occurred while creating the filter for event '{event_name}': {str(e)}")

    
def reset_attestation_chain(eth_address, contract):
    global nonce
    try:
        tx_data = contract.functions.ResetChain().buildTransaction({'from': eth_address, 'nonce': nonce})
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while calling the function: {str(e)}")
    
def register_agent(agent_uuid_str, eth_address, contract):
    global nonce
    try:
        uuid_obj = uuid.UUID(agent_uuid_str) 
        uuid_bytes16 = uuid_obj.bytes
        tx_data = contract.functions.RegisterAgent(
            uuid_bytes16
        ).buildTransaction({'from': eth_address, 'nonce': nonce})
        
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while registering agent: {str(e)}")


def remove_agent(eth_address, contract):
    global nonce
    try:
        tx_data = contract.functions.RemoveAgent().buildTransaction({'from': eth_address, 'nonce': nonce})
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while deleting agent: {str(e)}")


def is_prover_agent(attestation_id, eth_address, contract):
    try:
        result = contract.functions.IsProver(web3.toBytes(hexstr=attestation_id), eth_address).call()
        return result
    
    except Exception as e:
        raise Exception(f"An error occurred while getting the prover agent: {str(e)}")

def is_verifier_agent(attestation_id, eth_address, contract):
    try:
        result = contract.functions.IsVerifier(web3.toBytes(hexstr=attestation_id), eth_address).call()
        return result
    
    except Exception as e:
        raise Exception(f"An error occurred while getting the verifier agent: {str(e)}")
    
def send_fresh_signature(attestation_id, fresh_signature, eth_address, contract):
    global nonce
    try:
        tx_data = contract.functions.SendFreshSignaure(
             web3.toBytes(hexstr=attestation_id), 
             web3.toBytes(hexstr=f"0x{fresh_signature}") 
        ).buildTransaction({'from': eth_address, 'nonce': nonce})
        
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while sending the fresh signature: {str(e)}")

def send_reference_signature(attestation_id, reference_signature, eth_address, contract):
    global nonce
    try:
        tx_data = contract.functions.SendRefSignaure(
             web3.toBytes(hexstr=attestation_id), 
             web3.toBytes(hexstr=f"0x{reference_signature}") 
        ).buildTransaction({'from': eth_address, 'nonce': nonce})
        
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while sending the fresh signature: {str(e)}")
    
def get_attestation_measurements(attestation_id, eth_address, contract):
    try:
        fresh_signature, reference_signature = contract.functions.GetAttestationMeasurements(web3.toBytes(hexstr=attestation_id), eth_address).call()
        return web3.toHex(fresh_signature), web3.toHex(reference_signature)
    
    except Exception as e:
        raise Exception(f"An error occurred while getting the attestation measurements: {str(e)}")

def send_attestation_result(attestation_id, verified, eth_address, contract):
    global nonce
    try:
        tx_data = contract.functions.CloseAttestationProcess(
            web3.toBytes(hexstr=attestation_id),
            verified
        ).buildTransaction({'from': eth_address, 'nonce': nonce})
        
        tx_hash = send_signed_transaction(tx_data)
        return tx_hash
    
    except Exception as e:
        raise Exception(f"An error occurred while closing the attestation process: {str(e)}")

    
def get_agent_info(eth_address, contract):
    try:
        retrieved_uuid_bytes , is_registered, is_verified  = contract.functions.GetAgentInfo(eth_address).call()
        retrieved_uuid_str = uuid.UUID(bytes=retrieved_uuid_bytes).hex
        return retrieved_uuid_str, is_registered, is_verified
    
    except Exception as e:
        raise Exception(f"An error occurred while getting agent info: {str(e)}")

def get_prover_uuid(attestation_id, eth_address, contract):
    try:
        retrieved_uuid_bytes, prover_eth_address  = contract.functions.GetProverUUID(web3.toBytes(hexstr=attestation_id), eth_address).call()
        retrieved_uuid_str = uuid.UUID(bytes=retrieved_uuid_bytes).hex
        return retrieved_uuid_str
    
    except Exception as e:
        raise Exception(f"An error occurred while getting agent info: {str(e)}")
    
def get_attestation_info(attestation_id, contract):
    try:
        prover_address, verifier_address, attestation_result, timestamp  = contract.functions.GetAttestationInfo(web3.toBytes(hexstr=attestation_id)).call()
        return prover_address, verifier_address, attestation_result, timestamp
    
    except Exception as e:
        raise Exception(f"An error occurred while getting attestation info: {str(e)}")

def get_attestation_state(attestation_id, contract):
    try:
        attestation_state  = contract.functions.GetAttestationState(web3.toBytes(hexstr=attestation_id)).call()
        return attestation_state
    
    except Exception as e:
        raise Exception(f"An error occurred while getting attestation info: {str(e)}")

def display_attestation_state(attestation_id, contract):  
    current_attestation_state = get_attestation_state(attestation_id, contract)
    if current_attestation_state == 0:
        logger.info("Attestation state: Open")
    elif current_attestation_state == 1:
        logger.info("Attestation state: SecaasResponded")
    elif current_attestation_state == 2:
        logger.info("Attestation state: ProverResponded")
    elif current_attestation_state == 3:
        logger.info("Attestation state: ReadyForEvaluation")
    elif current_attestation_state == 4:
        logger.info("Attestation state: Closed")                
    else:
        logger.error(f"Error: state for attestation {attestation_id} is {current_attestation_state}")


def handle_secaas_logic(eth_address, contract, seen_events):
    """SECaaS logic: Attestation monitoring and verification process."""
    attestation_id = ''
    while True:

        # print("\SECaaS Menu:")
        # print("1. Clean attestation chain")
        # print("2. Read transaction receipt")
        # print("3. Continue...")

        # choice = input("Select an option (1-3): ").strip()

        # if not choice.isdigit():
        #     logger.info("Invalid input. Please enter a number between 1 and 3.")
        #     continue  # Re-prompt user

        # choice = int(choice)

        # try:
        #     if choice == 1:
        #         tx_hash = register_agent(agent_uuid, eth_address, contract)
        #         logger.info(f"Agent registered | Tx: {tx_hash}")

        #     elif choice == 2:
        #         tx_hash = input("Enter the transaction hash to retrieve the receipt: ").strip()
        #         if not tx_hash:
        #             logger.info("No transaction hash entered. Returning to menu.")
        #             continue

        #         receipt = get_transaction_receipt(tx_hash)
        #         logger.info("Transaction receipt details:")
        #         for key, value in receipt.items():
        #             print(f"{key}: {value}")
        #         break  # Exit after displaying receipt

        #     elif choice == 3:
        #         logger.info("Continuing to next step...")

        #     else:
        #         logger.info("Invalid option. Please select a number between 1 and 3.")

        # except Exception as e:
        #     logger.error(f"Error occurred: {str(e)}")


        logger.info("Subscribed to attestation events. Waiting for process initiation...")
        new_attestation_event_filter = create_event_filter(contract, MasMutualAttestationContractEvents.ATTESTATION_STARTED, 10)
        
        attestation_started = False
        while not attestation_started:
            attestation_events = new_attestation_event_filter.get_all_entries()
            for event in attestation_events:
                tx_hash = event['transactionHash'].hex()
                if tx_hash not in seen_events:
                    seen_events.add(tx_hash)
                    attestation_started = True
                    attestation_id = event["args"]["id"].hex()

                    logger.info(f"Attestation initiated:\n" + 
                        f"  Attestation ID: {attestation_id}\n" +
                        f"  Tx Hash: {tx_hash}\n")
                    break
        
        display_attestation_state(attestation_id, contract)

        logger.info(f"Retrieving UUID from SC...")
        agent_uuid = get_prover_uuid(attestation_id, eth_address, contract)
        print(f"Prover UUID: {agent_uuid}\n")

        logger.info(f"Retrieving reference signature from DB...")
        agent_info = utils.get_agent_by_uuid("/home/agent/config", agent_uuid)
        if not agent_info:
            logger.error(f"Agent with UUID {agent_uuid} not found in the DB.")
            continue
        
        reference_agent_measurement = agent_info['ref_signature']

        tx_hash = send_reference_signature(attestation_id, reference_agent_measurement, eth_address, contract)
        logger.info(f"Reference signature sent:\n" + 
            f"  Agent UUID: {agent_uuid}\n" +
            f"  Agent ref. signature: {agent_uuid}\n" +
            f"  Tx Hash: {tx_hash}\n")
        
        is_verifier = is_verifier_agent(attestation_id, eth_address, contract)
        if is_verifier:
            logger.info("You are the Verifier agent!")
            ready_for_evaluation = False 
            while ready_for_evaluation == False:
                ready_for_evaluation = True if get_attestation_state(attestation_id, contract) == 3 else False

            display_attestation_state(attestation_id, contract)
            
            # Retrieve and compare measurements
            fresh_signature, reference_signature = get_attestation_measurements(attestation_id, eth_address, contract)

            # Compare measurements
            logger.info("Comparing hashes...")
            result = fresh_signature == reference_signature

            # Save attestation result
            tx_hash = send_attestation_result(attestation_id, result, eth_address, contract)
            logger.info(f"Attestation closed:\n" + 
                f"  Result: {'SUCCESS' if result else 'FAILURE'}\n" +
                f"  Tx Hash: {tx_hash}\n")
            time.sleep(3)      


def handle_agent_logic(eth_address, contract, seen_events, agent_uuid, reference_measurement=None):
    """Handles Agent registration and attestation process."""
    attestation_id = ''
    while True:
        print("\nAgent Menu:")
        print("1. Register agent")
        print("2. Remove agent")
        print("3. Read transaction receipt")
        print("4. Continue...")

        choice = input("Select an option (1-4): ").strip()

        if not choice.isdigit():
            logger.info("Invalid input. Please enter a number between 1 and 4.")
            continue  # Re-prompt user

        choice = int(choice)

        try:
            if choice == 1:
                tx_hash = register_agent(agent_uuid, eth_address, contract)
                logger.info(f"Agent registered | Tx: {tx_hash}")

            elif choice == 2:
                tx_hash = remove_agent(eth_address, contract)
                logger.info(f"Agent removed | Tx: {tx_hash}")
                break  # Exit loop after removing agent

            elif choice == 3:
                tx_hash = input("Enter the transaction hash to retrieve the receipt: ").strip()
                if not tx_hash:
                    logger.info("No transaction hash entered. Returning to menu.")
                    continue

                receipt = get_transaction_receipt(tx_hash)
                logger.info("Transaction receipt details:")
                for key, value in receipt.items():
                    print(f"{key}: {value}")
                break  # Exit after displaying receipt

            elif choice == 4:
                logger.info("Continuing to next step...")

            else:
                logger.info("Invalid option. Please select a number between 1 and 4.")

        except Exception as e:
            logger.error(f"Error occurred: {str(e)}")

        logger.info("Subscribed to attestation events. Waiting for process initiation...")
        new_attestation_event_filter = create_event_filter(contract, MasMutualAttestationContractEvents.ATTESTATION_STARTED, 10)
        attestation_started = False
        while not attestation_started:
            attestation_events = new_attestation_event_filter.get_all_entries()
            for event in attestation_events:
                tx_hash = event['transactionHash'].hex()
                if tx_hash not in seen_events:
                    seen_events.add(tx_hash)
                    attestation_started = True
                    attestation_id = event["args"]["id"].hex()
                    logger.info(f"Attestation initiated:\n" + 
                        f"  Attestation ID: {attestation_id}\n" +
                        f"  Tx Hash: {tx_hash}\n")
                    break

        display_attestation_state(attestation_id, contract)

        # Check if the agent is the verifier
        is_verifier = is_verifier_agent(attestation_id, eth_address, contract)
        is_prover = is_prover_agent(attestation_id, eth_address, contract)

        if is_verifier:
            logger.info("You are the Verifier agent!")

            # Wait for secaas and prover to respond
            logger.info("Waiting for responses from SECaaS and Prover...")
            ready_for_evaluation = False 
            while ready_for_evaluation == False:
                ready_for_evaluation = True if get_attestation_state(attestation_id, contract) == 3 else False

            display_attestation_state(attestation_id, contract)

            # Retrieve and compare measurements
            fresh_signature, reference_signature = get_attestation_measurements(attestation_id, eth_address, contract)
            
            logger.info(f"Attestation measurements:\n" + 
                f"  Fresh Signature: {fresh_signature}\n" +
                f"  Reference: {reference_signature}\n")

            # Compare measurements
            logger.info("Comparing hashes...")
            result = fresh_signature == reference_signature

            # Save attestation result
            tx_hash = send_attestation_result(attestation_id, result, eth_address, contract)
            logger.info(f"Attestation closed:\n" + 
                f"  Result: {'SUCCESS' if result else 'FAILURE'}\n" +
                f"  Tx Hash: {tx_hash}\n")
            time.sleep(3)        
        
        # Check if the agent is the prover
        elif is_prover:
            logger.info("You are the Prover agent!")
            
            logger.info("Generating fresh signature...")
            time.sleep(2)

            tx_hash = send_fresh_signature(attestation_id, reference_measurement, eth_address, contract)            
            logger.info(f"Fresh signature sent:\n" + 
                f"  Signature: {reference_measurement}\n" +
                f"  Tx Hash: {tx_hash}\n")
            
            logger.info("Waiting for attestation to complete...")
            attestation_completed_event = create_event_filter(contract, MasMutualAttestationContractEvents.ATTESTATION_COMPLETED)
            attestation_completed = False
            while not attestation_completed:
                completed_events = attestation_completed_event.get_all_entries()
                for event in completed_events:
                    tx_hash = event['transactionHash'].hex()
                    if tx_hash not in seen_events:
                        seen_events.add(tx_hash)
                        attestation_completed = True
                        # print("\nAttestation process completed")
                        break
            
            display_attestation_state(attestation_id, contract)
            
            # Retrieve attestation details
            prover_address, verifier_address, attestation_result, timestamp = get_attestation_info(attestation_id, contract)
            
            logger.info(f"Attestation info:\n" + 
                f"  Prover address: {prover_address}\n" +
                f"  Verifier address: {verifier_address}\n" +
                f"  Result: {'SUCCESS' if attestation_result else 'FAILURE'}\n" +
                f"  Timestamp: {timestamp}\n")            
        else:
            logger.error("You are neither the Verifier nor the Prover, continuing to listen for events...")  
            time.sleep(3)
            continue

            
if __name__ == "__main__":
    try:
        if participant == 'secaas':
            handle_secaas_logic(eth_address, mas_contract, seen_events)

        elif participant == 'agent':
            handle_agent_logic(eth_address, mas_contract, seen_events, agent_uuid, reference_measurement)

    except Exception as e:
            print(f"An error occurred in the main execution: {str(e)}")
            sys.exit(1)





