from web3 import Web3
import logging
import uuid

def send_signed_transaction(build_transaction, private_key, web3_instance):
    """
    Sends a signed transaction to the blockchain network using the private key.
    Args:
        build_transaction (dict): The transaction data to be sent.
    Returns:
        str: The transaction hash of the sent transaction.
    """
    signed_txn = web3_instance.eth.account.signTransaction(build_transaction, private_key)
    tx_hash = web3_instance.eth.sendRawTransaction(signed_txn.rawTransaction)
    return tx_hash.hex()

def register_agent(agent_uuid_str, eth_address, contract, web3_instance=None, private_key=None):
    try:
        uuid_obj = uuid.UUID(agent_uuid_str) 
        uuid_bytes16 = uuid_obj.bytes
        logging.info(f"Registering agent with UUID: {agent_uuid_str}, bytes16: {uuid_bytes16.hex()}")
        nonce = web3_instance.eth.getTransactionCount(eth_address)
        tx_data = contract.functions.RegisterAgent(
            uuid_bytes16
        ).buildTransaction({'from': eth_address, 'nonce': nonce})
        logging.debug(f"Tx data: {tx_data}")
        tx_hash = send_signed_transaction(tx_data, private_key, web3_instance)
        logging.info(f"Agent registered with tx hash: {tx_hash}")
        return tx_hash
    except Exception as e:
        logging.error(f"An error occurred while registering agent: {str(e)}")
        raise Exception(f"An error occurred while registering agent: {str(e)}")



def handle_agent_logic(eth_address, contract, seen_events, agent_uuid, web3_instance=None, private_key=None, fail_attestation_flag=False, last_n_blocks=None):
    """
    Agent logic handler: can act as Verifier or Prover depending on the contract state.
    """
    # Vérifier si web3 et private_key sont disponibles
    if not web3_instance or not private_key:
        logging.error("web3_instance et private_key sont requis pour handle_agent_logic")
        return None

    # Vérifier si l'agent est déjà enregistré
    try:
        # GetAgentInfo(address agentAddress, address callAddress) returns (bytes16, bool, bytes32[] memory)
        agent_info = contract.functions.GetAgentInfo(eth_address, eth_address).call()
        is_registered = agent_info[1]
        # if is_registered:
        logging.info("Agent déjà enregistré, demande d'attestation...")
        tx_hash = request_attestation(eth_address, contract, private_key, web3_instance)
        logging.info(f"Attestation demandée avec tx hash: {tx_hash}")
        return tx_hash
    except Exception as e:
        raise Exception(f"Erreur lors de la vérification de l'enregistrement de l'agent: {str(e)}")

def request_attestation(eth_address, contract, private_key, web3_instance):
    try:
        logging.info(f"Requesting attestation for agent with address: {eth_address}")
        nonce = web3_instance.eth.getTransactionCount(eth_address)
        tx_data = contract.functions.RequestAttestation().buildTransaction({'from': eth_address, 'nonce': nonce})
        logging.debug(f"Tx data: {tx_data}")
        tx_hash = send_signed_transaction(tx_data, private_key, web3_instance)
        logging.info(f"Attestation requested with tx hash: {tx_hash}")
        return tx_hash
    except Exception as e:
        raise Exception(f"An error occurred while requesting attestation: {str(e)}")