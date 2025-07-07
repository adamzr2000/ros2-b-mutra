import logging
import sys
import argparse
from web3 import Web3, WebsocketProvider
from web3.middleware import geth_poa_middleware

# Initialize logging
logging.basicConfig(format='%(asctime)s - %(levelname)s - %(message)s', level=logging.INFO)
logger = logging.getLogger(__name__)

# Argument parser setup
parser = argparse.ArgumentParser(description="Retrieve Ethereum transaction receipt")
parser.add_argument('--eth-node-url', type=str, required=True,
                    help="Ethereum node WebSocket URL (e.g., ws://localhost:8546)")
parser.add_argument('--tx-hash', type=str, required=True,
                    help="Transaction hash to retrieve the receipt")

args = parser.parse_args()
eth_node_url, tx_hash = args.eth_node_url, args.tx_hash

# Initialize Web3
try:
    web3 = Web3(WebsocketProvider(eth_node_url))
    web3.middleware_onion.inject(geth_poa_middleware, layer=0)

    if not web3.isConnected():
        raise ConnectionError("Failed to connect to Ethereum node")

    logger.info(f"Connected to Ethereum node {eth_node_url} | Geth Version: {web3.clientVersion}")

except Exception as e:
    logger.error(f"Web3 initialization failed: {str(e)}")
    sys.exit(1)

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

    

# Fetch and display transaction receipt
receipt = get_transaction_receipt(tx_hash)
logger.info("Transaction Receipt Details:")
for key, value in receipt.items():
    print(f"{key}: {value}")