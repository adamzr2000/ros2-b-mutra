# blockchain_interface.py

import json
import uuid
import time
import logging
import threading
import warnings

from enum import IntEnum, Enum
from web3 import Web3, WebsocketProvider, HTTPProvider
from web3.middleware import geth_poa_middleware
from web3._utils.events import event_abi_to_log_topic

from prettytable import PrettyTable
from typing import Optional

logging.basicConfig(format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)
warnings.filterwarnings(
    "ignore",
    message="There was an issue with the method eth_maxPriorityFeePerGas",
)
class AttestationState(IntEnum):
    Open = 0
    ReadyForEvaluation = 1
    Closed = 2

class MasMutualAttestationContractEvents(str, Enum):
    AGENT_REGISTERED = "AgentRegistered"
    AGENT_REMOVED = "AgentRemoved"
    ATTESTATION_STARTED = "AttestationStarted"
    READY_FOR_EVALUATION = "ReadyForEvaluation"
    ATTESTATION_COMPLETED = "AttestationCompleted"

class BlockchainInterface:
    def __init__(self, eth_address, private_key, eth_node_url, abi_path, contract_address):
        if eth_node_url.startswith("ws://"):
            self.web3 = Web3(WebsocketProvider(eth_node_url))
        elif eth_node_url.startswith("http://"):
            self.web3 = Web3(HTTPProvider(eth_node_url))
        else:
            raise ValueError("eth_node_url must start with ws:// or http://")

        self.web3.middleware_onion.inject(geth_poa_middleware, layer=0)
        if not self.web3.isConnected():
            raise ConnectionError(f"Cannot connect to Ethereum node at {eth_node_url}")

        # --- Keys & Address ---
        self.private_key = private_key
        acct = self.web3.eth.account.from_key(self.private_key)
        derived_addr = acct.address  # checksum

        if eth_address:
            provided = Web3.toChecksumAddress(eth_address)
            if provided != derived_addr:
                logger.warning(
                    "Provided ETH_ADDRESS (%s) != address from private key (%s). Using derived.",
                    provided, derived_addr
                )
            self.eth_address = derived_addr
        else:
            self.eth_address = derived_addr


        with open(abi_path, "r") as f:
            abi = json.load(f).get("abi")
        if not abi:
            raise ValueError("ABI not found in JSON")

        self.contract = self.web3.eth.contract(address=Web3.toChecksumAddress(contract_address), abi=abi)

        logger.info(f"Web3 initialized. Address: {self.eth_address}")
        logger.info(f"Connected to Ethereum node {eth_node_url} | Version: {self.web3.clientVersion}")

        # Initialize local nonce and lock
        self._nonce_lock = threading.Lock()
        self._local_nonce = self.web3.eth.get_transaction_count(self.eth_address, 'pending')


    def send_signed_transaction(self, build_transaction):
        with self._nonce_lock:
            build_transaction['nonce'] = self._local_nonce
            self._local_nonce += 1

        # Bump the gas price slightly to avoid underpriced errors
        # If not using EIP-1559, inject legacy gasPrice
        if 'maxFeePerGas' not in build_transaction and 'maxPriorityFeePerGas' not in build_transaction:
            base_gas_price = self.web3.eth.gas_price
            build_transaction['gasPrice'] = int(base_gas_price * 1.25)

        # Else (EIP-1559): Optional tweak to bump the maxFeePerGas slightly
        elif 'maxFeePerGas' in build_transaction:
            build_transaction['maxFeePerGas'] = int(build_transaction['maxFeePerGas'] * 1.25)
            
        # print(f"nonce = {build_transaction['nonce']}, maxFeePerGas = {build_transaction['maxFeePerGas']}")
        signed_txn = self.web3.eth.account.signTransaction(build_transaction, self.private_key)
        tx_hash = self.web3.eth.sendRawTransaction(signed_txn.rawTransaction)
        return tx_hash.hex()


    def get_event_class(self, event_enum: MasMutualAttestationContractEvents):
        """
        Returns the web3 ContractEvent class for the given enum (e.g., contract.events.AttestationStarted)
        """
        return getattr(self.contract.events, event_enum.value)

    def get_event_abi(self, event_enum: MasMutualAttestationContractEvents) -> dict:
        """
        Returns the ABI dict for the event (used by the watcher to decode logs).
        """
        return self.get_event_class(event_enum)._get_event_abi()

    def get_event_topic(self, event_enum: MasMutualAttestationContractEvents) -> str:
        """
        Returns keccak(signature) topic for the event (topic0) from its ABI.
        Compatible with web3.py==5.31.
        """
        abi = self.get_event_abi(event_enum)
        return event_abi_to_log_topic(abi).hex()
    
    def get_transaction_receipt(self, tx_hash: str) -> dict:
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
            receipt = self.web3.eth.get_transaction_receipt(tx_hash)

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
                block = self.web3.eth.get_block(block_number)

                # Add the block timestamp to the receipt dictionary
                receipt_dict['timestamp'] = block['timestamp']

                return receipt_dict

            else:
                raise Exception("Error: Transaction receipt not found")

        except Exception as e:
            raise Exception(f"An exception occurred: {str(e)}")


    def create_event_filter(self, event_name: MasMutualAttestationContractEvents, last_n_blocks: int = None):
        """
        Creates a filter for the given contract event. Robust against empty chains
        and node warm-up by avoiding getBlock('latest').
        """
        try:
            # Get current chain height safely
            current_height = self.web3.eth.block_number  # returns int, never raises BlockNotFound
            # If node is brand new, current_height can be 0 (genesis)
            if current_height is None or current_height < 0:
                current_height = 0

            if last_n_blocks is not None and last_n_blocks > 0:
                from_block = max(0, current_height - last_n_blocks)
            else:
                # Start at 0 so we won't miss very early events if the app races the node
                from_block = 0

            # Build the event class
            event_cls = getattr(self.contract.events, event_name.value, None)
            if event_cls is None:
                raise ValueError(f"Event '{event_name}' does not exist in the contract ABI.")

            # IMPORTANT: pass integers, not hex strings, for web3.py v5
            # Also add toBlock='latest' for clarity
            event_filter = event_cls.createFilter(fromBlock=from_block, toBlock='latest')
            return event_filter

        except ValueError as ve:
            # wrong event name / ABI mismatch
            raise ve
        except Exception as e:
            # As a last resort, start from 'earliest'
            try:
                event_cls = getattr(self.contract.events, event_name.value)
                return event_cls.createFilter(fromBlock=0, toBlock='latest')
            except Exception as inner:
                raise Exception(f"Failed to create filter for event '{event_name}': {inner}") from e
    
    def reset_attestation_chain(self):
        try:
            tx_data = self.contract.functions.ResetChain().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while calling the function: {str(e)}")

    def request_attestation(self, attestation_id, fresh_signature):
        try:
            tx_data = self.contract.functions.RequestAttestation(
                self.web3.toBytes(text=attestation_id), 
                self.web3.toBytes(hexstr=f"0x{fresh_signature}") 
            ).buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while requesting attestation: {str(e)}")
    
    def register_agent(self, agent_uuid_str, wait: bool = False, timeout: int = 120):
        try:
            uuid_obj = uuid.UUID(agent_uuid_str) 
            uuid_bytes16 = uuid_obj.bytes
            tx_data = self.contract.functions.RegisterAgent(
                uuid_bytes16
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            if wait:
                logger.info(f"Waiting for transaction {tx_hash} to be mined...")
                receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

                if receipt.status != 1:
                    raise Exception(
                        f"Transaction {tx_hash} was mined in block {receipt.blockNumber} but failed (status=0)."
                    )

                logger.info(f"Transaction {tx_hash} successfully included in block {receipt.blockNumber}")

            return tx_hash

        except Exception as e:
            logger.error(f"Failed to register domain: {str(e)}")
            raise Exception(f"Failed to register domain: {str(e)}")
        

    def remove_agent(self, wait: bool = False, timeout: int = 120):
        try:
            tx_data = self.contract.functions.RemoveAgent().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            if wait:
                logger.info(f"Waiting for transaction {tx_hash} to be mined...")
                receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

                if receipt.status != 1:
                    raise Exception(
                        f"Transaction {tx_hash} was mined in block {receipt.blockNumber} but failed (status=0)."
                    )

                logger.info(f"Transaction {tx_hash} successfully included in block {receipt.blockNumber}")

            return tx_hash
        
        except Exception as e:
            logger.error(f"Failed to unregister domain: {str(e)}")
            raise Exception(f"Failed to unregister domain: {str(e)}")
        
    def is_prover_agent(self, attestation_id):
        try:
            result = self.contract.functions.IsProver(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            return result
        
        except Exception as e:
            raise Exception(f"An error occurred while getting the prover agent: {str(e)}")

    def is_verifier_agent(self, attestation_id):
        try:
            result = self.contract.functions.IsVerifier(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            return result
        
        except Exception as e:
            raise Exception(f"An error occurred while getting the verifier agent: {str(e)}")
    
    def send_reference_signature(self, attestation_id, reference_signature):
        try:
            tx_data = self.contract.functions.SendRefSignaure(
                self.web3.toBytes(text=attestation_id), 
                self.web3.toBytes(hexstr=f"0x{reference_signature}") 
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while sending the reference signature: {str(e)}")
    
    def get_attestation_measurements(self, attestation_id):
        try:
            fresh_signature, reference_signature = self.contract.functions.GetAttestationMeasurements(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            return self.web3.toHex(fresh_signature), self.web3.toHex(reference_signature)
        
        except Exception as e:
            raise Exception(f"An error occurred while getting the attestation measurements: {str(e)}")
        
    def send_attestation_result(self, attestation_id, verified):
        try:
            tx_data = self.contract.functions.CloseAttestationProcess(
                self.web3.toBytes(text=attestation_id),
                verified
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while closing the attestation process: {str(e)}")
        
    def get_agent_info(self, agent_address):
        try:
            retrieved_uuid_bytes , is_registered, completed_attestations  = self.contract.functions.GetAgentInfo(agent_address, self.eth_address).call()
            retrieved_uuid_str = uuid.UUID(bytes=retrieved_uuid_bytes).hex
            return retrieved_uuid_str, is_registered, completed_attestations
        
        except Exception as e:
            raise Exception(f"An error occurred while getting agent info: {str(e)}")

    def get_prover_uuid(self, attestation_id):
        try:
            retrieved_uuid_bytes  = self.contract.functions.GetProverUUID(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            retrieved_uuid_str = uuid.UUID(bytes=retrieved_uuid_bytes).hex
            return retrieved_uuid_str
        
        except Exception as e:
            raise Exception(f"An error occurred while getting agent info: {str(e)}")

    def get_attestation_chain(self):
        try:
            attestation_chain = self.contract.functions.GetAttestationChain().call()
            return attestation_chain
        
        except Exception as e:
            raise Exception(f"An error occurred while getting the attestation chain: {str(e)}")
      
    def get_attestation_info(self, attestation_id):
        try:
            prover_address, verifier_address, attestation_result, timestamp  = self.contract.functions.GetAttestationInfo(self.web3.toBytes(text=attestation_id)).call()
            return prover_address, verifier_address, attestation_result, timestamp
        
        except Exception as e:
            raise Exception(f"An error occurred while getting attestation info: {str(e)}")

    def get_attestation_state(self, attestation_id):
        try:
            attestation_state  = self.contract.functions.GetAttestationState(self.web3.toBytes(text=attestation_id)).call()
            return attestation_state
        
        except Exception as e:
            raise Exception(f"An error occurred while getting attestation info: {str(e)}")

    def display_attestation_state(self, attestation_id):  
        current_attestation_state = self.get_attestation_state(attestation_id)
        try:
            state_name = AttestationState(current_attestation_state).name
            logger.info(f"Attestation state: {state_name}")
        except ValueError:
            logger.error(f"Unknown attestation state: {current_attestation_state}")

    def display_attestation_chain(self, last_n=10):
        attestation_chain = self.get_attestation_chain()

        if not attestation_chain:
            print("=== Attestation chain is empty ===")
            return

        print(f"=== Last {min(last_n, len(attestation_chain))} Attestations ===")

        table = PrettyTable()
        table.field_names = ["#", "Attestation ID", "Prover", "Verifier", "Result", "Timestamp"]
        table.align = "l"

        # Get only the last `last_n` attestations
        recent_chain = attestation_chain[-last_n:]

        for i, attestation_id in enumerate(recent_chain, start=1):
            attestation_id = attestation_id.rstrip(b'\x00').decode('utf-8')
            prover_address, verifier_address, attestation_result, timestamp = self.get_attestation_info(attestation_id)
            status = "✅ SUCCESS" if attestation_result == 2 else "❌ FAILURE"
            table.add_row([i, attestation_id, prover_address, verifier_address, status, timestamp])

        print(table)
        print()