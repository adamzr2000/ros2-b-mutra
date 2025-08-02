# blockchain_interface.py

import json
import uuid
import time
import logging
from enum import Enum
from web3 import Web3, WebsocketProvider, HTTPProvider
from web3.middleware import geth_poa_middleware
from prettytable import PrettyTable

logging.basicConfig(format='%(asctime)s - %(levelname)s - [%(threadName)s] %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)


class MasMutualAttestationContractEvents(str, Enum):
    AGENT_REGISTERED = "AgentRegistered"
    AGENT_REMOVED = "AgentRemoved"
    ATTESTATION_STARTED = "AttestationStarted"
    SECAAS_RESPONDED = "SecaasResponded"
    PROVER_RESPONDED = "ProverResponded"
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

        self.eth_address = eth_address
        self.private_key = private_key

        with open(abi_path, "r") as f:
            abi = json.load(f).get("abi")
        if not abi:
            raise ValueError("ABI not found in JSON")

        self.contract = self.web3.eth.contract(address=Web3.toChecksumAddress(contract_address), abi=abi)

        logger.info(f"Web3 initialized. Address: {self.eth_address}")
        logger.info(f"Connected to Ethereum node {eth_node_url} | Version: {self.web3.clientVersion}")


    def send_signed_transaction(self, build_transaction):
        nonce = self.web3.eth.getTransactionCount(self.eth_address, 'pending')
        build_transaction['nonce'] = nonce

        # Bump the gas price slightly to avoid underpriced errors
        # If not using EIP-1559, inject legacy gasPrice
        if 'maxFeePerGas' not in build_transaction and 'maxPriorityFeePerGas' not in build_transaction:
            base_gas_price = self.web3.eth.gas_price
            build_transaction['gasPrice'] = int(base_gas_price * 1.1)

        # Else (EIP-1559): Optional tweak to bump the maxFeePerGas slightly
        elif 'maxFeePerGas' in build_transaction:
            build_transaction['maxFeePerGas'] = int(build_transaction['maxFeePerGas'] * 1.1)

        signed_txn = self.web3.eth.account.signTransaction(build_transaction, self.private_key)
        tx_hash = self.web3.eth.sendRawTransaction(signed_txn.rawTransaction)
        return tx_hash.hex()

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
            block = self.web3.eth.getBlock('latest')
            block_number = block['number']
            
            # If last_n_blocks is provided, look back, otherwise start from the latest block
            from_block = max(0, block_number - last_n_blocks) if last_n_blocks else block_number
            
            # Use the contract instance passed as an argument to access the events
            event_filter = getattr(self.contract.events, event_name.value).createFilter(fromBlock=self.web3.toHex(from_block))
            return event_filter
        except AttributeError:
            raise ValueError(f"Event '{event_name}' does not exist in the contract.")
        except Exception as e:
            raise Exception(f"An error occurred while creating the filter for event '{event_name}': {str(e)}")

    
    def reset_attestation_chain(self):
        try:
            tx_data = self.contract.functions.ResetChain().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while calling the function: {str(e)}")

    def request_attestation(self, attestation_id):
        try:
            tx_data = self.contract.functions.RequestAttestation(id=self.web3.toBytes(text=attestation_id)).buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while requesting attestation: {str(e)}")
    
    def register_agent(self, agent_uuid_str):
        try:
            uuid_obj = uuid.UUID(agent_uuid_str) 
            uuid_bytes16 = uuid_obj.bytes
            tx_data = self.contract.functions.RegisterAgent(
                uuid_bytes16
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while registering agent: {str(e)}")


    def remove_agent(self):
        try:
            tx_data = self.contract.functions.RemoveAgent().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while deleting agent: {str(e)}")


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
    
    def send_fresh_signature(self, attestation_id, fresh_signature):
        try:
            tx_data = self.contract.functions.SendFreshSignaure(
                self.web3.toBytes(text=attestation_id), 
                self.web3.toBytes(hexstr=f"0x{fresh_signature}") 
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while sending the fresh signature: {str(e)}")

    def send_reference_signature(self, attestation_id, reference_signature):
        try:
            tx_data = self.contract.functions.SendRefSignaure(
                self.web3.toBytes(text=attestation_id), 
                self.web3.toBytes(hexstr=f"0x{reference_signature}") 
            ).buildTransaction({'from': self.eth_address})
            
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while sending the fresh signature: {str(e)}")
    
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