from web3 import Web3
import json

json_file_path = "/home/agent/config/Secaas.json"

try:
    # Read the JSON data from the file
    with open(json_file_path, "r") as json_file:
        data = json.load(json_file)

    eth_address = data["eth_address"]
    private_key = data["private_key"]
    eth_node_url = data["eth_node"]
    contract_address = Web3.toChecksumAddress(data["contract_address"])

except FileNotFoundError:
    print(f"Error: JSON configuration file not found at {json_file_path}.")
except KeyError as e:
    print(f"Error: Missing expected key {e} in the configuration file.")
except Exception as e:
    print(f"Unexpected error occurred: {str(e)}")


# Replace with the correct path to your contract ABI JSON file
ABI_FILE_PATH = "/home/agent/smart-contracts/build/contracts/MasMutualAttestation.json"

# Initialize Web3 connection using WebSockets
try:
    web3 = Web3(Web3.WebsocketProvider(eth_node_url))

    if web3.isConnected():
        print(f"✅ Connected to Ethereum WebSocket node: {eth_node_url}")
    else:
        raise ConnectionError(f"❌ Failed to connect to Ethereum WebSocket node: {eth_node_url}")

    # Load contract ABI
    with open(ABI_FILE_PATH, "r") as abi_file:
        contract_abi = json.load(abi_file).get("abi", None)

    if not contract_abi:
        raise ValueError("❌ Error: ABI not found in the JSON file")

    # Create contract instance
    contract = web3.eth.contract(address=Web3.toChecksumAddress(contract_address), abi=contract_abi)
    print(f"✅ Contract loaded successfully at address: {contract_address}")

except Exception as e:
    print(f"❌ Error: {str(e)}")
