import json
import os
import hashlib
import math
import uuid
import argparse
import random

BESU_BASE = "blockchain/quorum-test-network"
NODES_DIR = os.path.join(BESU_BASE, "config", "nodes")

# Random RPC URL pool
VALIDATOR_RPC_POOL = [
    "http://validator1:8545",
    "http://validator2:8545",
    "http://validator3:8545",
    "http://validator4:8545",
]

# Deployed contract info
with open('smart-contracts/deployments/besu-MasMutualAttestation.json') as f:
    contract_data = json.load(f)

def pick_random_eth_node_url() -> str:
    return random.choice(VALIDATOR_RPC_POOL)

def read_credentials_from_dir(node_dir: str, eth_node_url: str):
    keystore_path = os.path.join(node_dir, "accountKeystore")
    pk_path = os.path.join(node_dir, "accountPrivateKey")

    if not os.path.isdir(node_dir):
        raise FileNotFoundError(f"Node directory not found: {node_dir}")
    if not os.path.isfile(keystore_path):
        raise FileNotFoundError(f"Missing keystore: {keystore_path}")
    if not os.path.isfile(pk_path):
        raise FileNotFoundError(f"Missing private key: {pk_path}")

    with open(keystore_path, "r") as f:
        keystore = json.load(f)
    raw_addr = (keystore.get("address") or "").strip()
    if not raw_addr:
        raise ValueError(f"Keystore missing 'address' field: {keystore_path}")

    eth_address = raw_addr if raw_addr.startswith("0x") else f"0x{raw_addr}"

    with open(pk_path, "r") as f:
        private_key = f.read().strip()

    return {
        "eth_address": eth_address,
        "private_key": private_key,
        "eth_node_url": eth_node_url,
    }

def generate_fake_hash(data: str) -> str:
    return hashlib.sha256(data.encode("utf-8")).hexdigest()

def create_agent_config(agent_index: int, output_directory: str):
    # robot i -> agent i
    agent_dir = os.path.join(NODES_DIR, f"agent{agent_index}")
    eth_node_url = pick_random_eth_node_url()
    creds = read_credentials_from_dir(agent_dir, eth_node_url)

    # TO BE REPLACED
    fake_hash = generate_fake_hash(f"robot{agent_index}")

    agent_data = {
        "name": f"robot{agent_index}",
        "cmd_name": "robot_state_publisher",
	    "text_section_size": 42223,
        "sha256": "6ca591b61f86f966abdfafdc50547b241072696c93173188c302e6e64f432566",
	    "offset": 0, 
        "eth_address": creds["eth_address"],         # from keystore, with 0x
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],       # random validator URL
        "contract_address": contract_data["address"],
        "ref_signatures": [fake_hash, fake_hash, fake_hash], # [robot, prover, verifier]
    }

    os.makedirs(output_directory, exist_ok=True)
    filepath = os.path.join(output_directory, f"robot{agent_index}.json")
    with open(filepath, "w") as f:
        json.dump(agent_data, f, indent=4)

    print(f"Created robot{agent_index}.json using agent{agent_index} (RPC: {eth_node_url}).")

def create_secaas_config(output_directory: str):
    # MUST read from validator1
    validator_dir = os.path.join(NODES_DIR, "validator1")
    eth_node_url = pick_random_eth_node_url()
    creds = read_credentials_from_dir(validator_dir, eth_node_url)

    secaas_data = {
        "name": "secaas",
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],       # random validator URL
        "contract_address": contract_data["address"],
    }

    os.makedirs(output_directory, exist_ok=True)
    filepath = os.path.join(output_directory, "secaas.json")
    with open(filepath, "w") as f:
        json.dump(secaas_data, f, indent=4)

    print(f"Created secaas.json using validator1 (RPC: {eth_node_url}).")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create agent configs from Besu quickstart files.")
    parser.add_argument("--num-agents", type=int, required=True, help="Number of agents to create")
    parser.add_argument("--output", default="./config", help="Output directory for JSON files")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    # Create secaas from validator1
    create_secaas_config(args.output)

    # Create robots: robot i -> agent i
    for i in range(1, args.num_agents + 1):
        create_agent_config(i, args.output)

    print(f"Successfully created {args.num_agents} agent configurations in {args.output}.")
