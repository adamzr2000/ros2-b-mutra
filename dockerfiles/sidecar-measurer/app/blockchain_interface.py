# blockchain_interface.py

import json
import time
import logging
from enum import Enum
from web3 import Web3, WebsocketProvider, HTTPProvider
from web3.middleware import geth_poa_middleware

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
        # print(build_transaction)
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

    def request_attestation(self, attestation_id):
        try:
            tx_data = self.contract.functions.RequestAttestation(id=self.web3.toBytes(text=attestation_id)).buildTransaction({'from': self.eth_address})
            tx_hash = self.send_signed_transaction(tx_data)
            return tx_hash
        
        except Exception as e:
            raise Exception(f"An error occurred while requesting attestation: {str(e)}")

