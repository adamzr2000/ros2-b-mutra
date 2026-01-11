# self.py

import json
import time
import logging
import threading
import warnings
import os

from enum import IntEnum, Enum
from web3 import Web3, WebsocketProvider, HTTPProvider
from web3.middleware import geth_poa_middleware
from web3._utils.events import event_abi_to_log_topic
from web3.exceptions import ContractLogicError

from prettytable import PrettyTable
from typing import Iterable, Optional, Tuple, Dict, Any

logger = logging.getLogger(__name__)
warnings.filterwarnings(
    "ignore",
    message="There was an issue with the method eth_maxPriorityFeePerGas",
)

def _as_bytes32(w3: Web3, v) -> bytes:
    """
    Accepts hex string (with or without 0x) or raw 32-byte value and returns bytes32.
    Raises ValueError if not 32 bytes after parsing.
    """
    if isinstance(v, (bytes, bytearray)):
        if len(v) != 32:
            raise ValueError(f"bytes32 must be 32 bytes, got {len(v)}")
        return bytes(v)
    if isinstance(v, str):
        hs = v if v.startswith("0x") else f"0x{v}"
        b = w3.toBytes(hexstr=hs)
        if len(b) != 32:
            raise ValueError(f"bytes32 must be 32 bytes, got {len(b)} after parsing '{v}'")
        return b
    raise TypeError(f"Unsupported type for bytes32: {type(v)}")

def _as_bytes32_triplet(w3: Web3, values) -> list:
    if values is None or len(values) != 3:
        raise ValueError("Expected 3 signatures: [robot, prover, verifier]")
    return [_as_bytes32(w3, values[0]), _as_bytes32(w3, values[1]), _as_bytes32(w3, values[2])]

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

class BlockchainManager:
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

        logger.info("Web3 client initialized:")
        logger.info(f"  - Address               : {self.eth_address}")
        logger.info(f"  - Ethereum node url     : {eth_node_url}")
        logger.info(f"  - Ethereum node version : {self.web3.clientVersion}")

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
    
    def reset_attestation_chain(self, wait: bool = False, timeout: int = 120):
        try:
            tx_data = self.contract.functions.ResetChain().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            
            if wait:
                logger.debug(f"Waiting for transaction {tx_hash} to be mined...")
                receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

                if receipt.status != 1:
                    raise Exception(
                        f"Transaction {tx_hash} was mined in block {receipt.blockNumber} but failed (status=0)."
                    )

                logger.debug(f"Transaction {tx_hash} successfully included in block {receipt.blockNumber}")

            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while calling the function: {str(e)}")

    def send_evidence(self, attestation_id, fresh_signatures):
        try:
            fresh_signatures_bytes32 = _as_bytes32_triplet(self.web3, fresh_signatures)
            tx_data = self.contract.functions.SendEvidence(
                self.web3.toBytes(text=attestation_id), 
                fresh_signatures_bytes32  # bytes32[3]
            ).buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while requesting attestation: {str(e)}")
    
    def register_agent(self, name, wait: bool = False, timeout: int = 120):
        try:
            tx_data = self.contract.functions.RegisterAgent(name).buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            if wait:
                logger.debug(f"Waiting for transaction {tx_hash} to be mined...")
                receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

                if receipt.status != 1:
                    raise Exception(
                        f"Transaction {tx_hash} was mined in block {receipt.blockNumber} but failed (status=0)."
                    )

                logger.debug(f"Transaction {tx_hash} successfully included in block {receipt.blockNumber}")

            return tx_hash

        except Exception as e:
            logger.error(f"Failed to register domain: {str(e)}")
            raise Exception(f"Failed to register domain: {str(e)}")
        

    def remove_agent(self, wait: bool = False, timeout: int = 120):
        try:
            tx_data = self.contract.functions.RemoveAgent().buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            if wait:
                logger.debug(f"Waiting for transaction {tx_hash} to be mined...")
                receipt = self.web3.eth.wait_for_transaction_receipt(tx_hash)

                if receipt.status != 1:
                    raise Exception(
                        f"Transaction {tx_hash} was mined in block {receipt.blockNumber} but failed (status=0)."
                    )

                logger.debug(f"Transaction {tx_hash} successfully included in block {receipt.blockNumber}")

            return tx_hash
        
        except Exception as e:
            logger.error(f"Failed to unregister domain: {str(e)}")
            raise Exception(f"Failed to unregister domain: {str(e)}")

    def is_registered(self):
        return self.contract.functions.IsRegistered(self.eth_address).call()
        
    def is_prover_agent(self, attestation_id):
        att_id = self.web3.toBytes(text=attestation_id)

        # If closed, contract would revert in IsProver => treat as "not prover" (or "not allowed")
        state = self.contract.functions.GetAttestationState(att_id).call()
        if state == AttestationState.Closed:
            return False

        try:
            return self.contract.functions.IsProver(att_id, self.eth_address).call()
        except ContractLogicError as e:
            # covers revert reasons like "closed or not exists"
            logger.info("IsProver reverted for %s: %s", attestation_id, e)
            return False


    def is_verifier_agent(self, attestation_id):
        att_id = self.web3.toBytes(text=attestation_id)

        state = self.contract.functions.GetAttestationState(att_id).call()
        if state == AttestationState.Closed:
            return False

        try:
            return self.contract.functions.IsVerifier(att_id, self.eth_address).call()
        except ContractLogicError as e:
            logger.info("IsVerifier reverted for %s: %s", attestation_id, e)
            return False
    

    def send_reference_signatures(self, attestation_id, ref_signatures, wait=False):
        try:
            ref_signatures_bytes32 = _as_bytes32_triplet(self.web3, ref_signatures)

            tx_data = self.contract.functions.SendRefSignaures(
                self.web3.toBytes(text=attestation_id),
                ref_signatures_bytes32
            ).buildTransaction({'from': self.eth_address})

            tx_hash = self.send_signed_transaction(tx_data)
            
            if wait:
                self.web3.eth.wait_for_transaction_receipt(tx_hash) # Block until mined
                
            return tx_hash

        except Exception as e:
            raise Exception(f"An error occurred while sending the reference signature: {str(e)}")

    def get_attestation_signatures(self, attestation_id):
        try:
            fresh_signatures_bytes32, ref_signatures_bytes32 = self.contract.functions.GetAttestationSignatures(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            
            # Convert to hex strings
            fresh_signatures = [self.web3.toHex(x) for x in list(fresh_signatures_bytes32)]
            ref_signatures   = [self.web3.toHex(x) for x in list(ref_signatures_bytes32)]

            return fresh_signatures, ref_signatures
        
        except Exception as e:
            raise Exception(f"An error occurred while getting the attestation signatures: {str(e)}")
        
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
            name, is_registered, completed_attestations  = self.contract.functions.GetAgentInfo(agent_address, self.eth_address).call()
            return name, is_registered, completed_attestations
        
        except Exception as e:
            raise Exception(f"An error occurred while getting agent info: {str(e)}")

    def get_prover_address(self, attestation_id):
        try:
            prover_address  = self.contract.functions.GetProverAddress(self.web3.toBytes(text=attestation_id), self.eth_address).call()
            return prover_address
        
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


    def _safe_display(self, chain_display_n):
        try:
            # Use logger to avoid stdout buffering issues
            attestation_chain = self.get_attestation_chain()
            if not attestation_chain:
                logger.info("=== Attestation chain is empty ===")
                return

            last_n = min(chain_display_n, len(attestation_chain))
            logger.info(f"=== Last {last_n} Attestations ===")

            table = PrettyTable()
            table.field_names = ["#", "Attestation ID", "Prover", "Verifier", "Result", "Timestamp"]
            table.align = "l"

            recent_chain = attestation_chain[-last_n:]
            for i, attestation_id in enumerate(recent_chain, start=1):
                attestation_id = attestation_id.rstrip(b'\x00').decode('utf-8')
                prover_address, verifier_address, attestation_result, timestamp = self.get_attestation_info(attestation_id)
                status = "✅ SUCCESS" if attestation_result == 2 else "❌ FAILURE"
                prover_name, _, _ = self.get_agent_info(prover_address)
                verifier_name = ""
                if Web3.toChecksumAddress(verifier_address) == Web3.toChecksumAddress("0xed9d02e382b34818e88b88a309c7fe71e65f419d"):
                    verifier_name = "SECaaS"
                else:
                    verifier_name, _, _ = self.get_agent_info(verifier_address)

                prover_info = f"{prover_name} - {prover_address}"
                verifier_info = f"{verifier_name} - {verifier_address}"
                table.add_row([i, attestation_id, prover_info, verifier_info, status, timestamp])

            logger.info("\n" + table.get_string())
        except Exception as e:
            logger.exception(f"Failed to display attestation chain: {e}")

    def periodic_attestation_chain_display(self, chain_display_n: int, interval: int, run_immediately: bool = True):
        if run_immediately:
            self._safe_display(chain_display_n)
        while True:
            time.sleep(interval)
            self._safe_display(chain_display_n)

class EventWatcher:
    def __init__(
        self,
        web3: Web3,
        contract,
        event_abi,                   # contract.events.EventName._get_event_abi()
        address: str,
        topics: Optional[list] = None,
        checkpoint_path: str = "event_checkpoint.json",
        confirmations: int = 2,
        batch_size: int = 1000,
        poll_interval: float = 1.0,
    ):
        self.w3 = web3
        self.address = Web3.toChecksumAddress(address)
        self.contract = contract
        self.event_abi = event_abi
        self.topics = topics
        self.checkpoint_path = checkpoint_path
        self.confirmations = confirmations
        self.batch_size = batch_size
        self.poll_interval = poll_interval

        cp = self._load_checkpoint()
        self.from_block = cp.get("from_block", self.w3.eth.block_number)
        self.seen_keys = set(cp.get("seen_keys", []))  # list of "blockHash-logIndex"

    def _load_checkpoint(self) -> Dict[str, Any]:
        if os.path.exists(self.checkpoint_path):
            with open(self.checkpoint_path, "r") as f:
                return json.load(f)
        return {}

    def _save_checkpoint(self):
        # ensure parent dir exists
        os.makedirs(os.path.dirname(self.checkpoint_path) or ".", exist_ok=True)
        with open(self.checkpoint_path, "w") as f:
            json.dump(
                {"from_block": self.from_block, "seen_keys": list(self.seen_keys)},
                f,
                indent=2,
            )

    def _latest_safe_block(self) -> int:
        return max(0, self.w3.eth.block_number - self.confirmations)

    def _unique_key(self, log) -> str:
        return f"{log['blockHash'].hex()}-{log['logIndex']}"

    def _decode_log(self, raw_log):
        return self.contract.events[self.event_abi['name']]().processLog(raw_log)

    def _fetch_logs_range(self, start_block: int, end_block: int) -> Iterable[dict]:
        # raw logs via eth_getLogs (address+topics filter)
        return self.w3.eth.getLogs({
            "fromBlock": start_block,
            "toBlock": end_block,
            "address": self.address,
            "topics": self.topics,
        })

    def run(self, handle_event):
        event_name = self.event_abi.get("name", "UnknownEvent")
        topics_str = ",".join(self.topics) if self.topics else "None"

        logger.info(
            "EventWatcher start: event=%s address=%s from_block=%d confirmations=%d batch_size=%d poll_interval=%.3fs topics=%s",
            event_name,
            self.address,
            self.from_block,
            self.confirmations,
            self.batch_size,
            self.poll_interval,
            topics_str,
        )
        while True:
            try:
                safe_to = self._latest_safe_block()
                if self.from_block > safe_to:
                    time.sleep(self.poll_interval)
                    continue

                while self.from_block <= safe_to:
                    to_block = min(self.from_block + self.batch_size - 1, safe_to)
                    raw_logs = self._fetch_logs_range(self.from_block, to_block)

                    # deterministic ordering
                    raw_logs.sort(key=lambda l: (l['blockNumber'], l['logIndex']))

                    for raw in raw_logs:
                        key = self._unique_key(raw)
                        if key in self.seen_keys:
                            continue
                        evt = self._decode_log(raw)
                        handle_event(evt)  # your business logic
                        self.seen_keys.add(key)

                    self.from_block = to_block + 1
                    self._save_checkpoint()

                time.sleep(self.poll_interval)

            except Exception as e:
                logger.exception(f"Watcher error, backing off: {e}")
                time.sleep(3)