import json
import os
import hashlib
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
with open("smart-contracts/deployments/besu-MasMutualAttestation.json") as f:
    contract_data = json.load(f)

# Contract ABI (Hardhat artifact)
ABI_PATH = "smart-contracts/artifacts/contracts/MasMutualAttestation.sol/MasMutualAttestation.json"
with open(ABI_PATH, "r") as f:
    artifact = json.load(f)

if "abi" not in artifact:
    raise KeyError(f"ABI not found in artifact: {ABI_PATH}")

CONTRACT_ABI = artifact["abi"]

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

def create_agent_files(agent_index: int, config_dir: str, ref_dir: str):
    # robot i -> agent i
    agent_dir = os.path.join(NODES_DIR, f"agent{agent_index}")
    eth_node_url = pick_random_eth_node_url()
    creds = read_credentials_from_dir(agent_dir, eth_node_url)

    # TO BE REPLACED (placeholder reference signatures)
    fake_hash = generate_fake_hash(f"robot{agent_index}")

    # 1) Normal config (secaas-like) -> ./config/robot{i}.json
    robot_config = {
        "name": f"robot{agent_index}",
        "cmd_name": "robot_state_publisher",
        "text_section_size": 42223,
        "offset": 0,
        "text_section_prefix": "",
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],
        "contract_address": contract_data["address"],
        "contract_abi": CONTRACT_ABI,
    }

    os.makedirs(config_dir, exist_ok=True)
    config_path = os.path.join(config_dir, f"robot{agent_index}.json")
    with open(config_path, "w") as f:
        json.dump(robot_config, f, indent=4)

    # 2) Ref-measurements -> ./ref-measurements/robot{i}.json
    ref_measurements = {
        "name": f"robot{agent_index}",
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "robot_hash": fake_hash,
        "attestation_sidecar_hash": fake_hash,
        "combined_hash": fake_hash
    }

    os.makedirs(ref_dir, exist_ok=True)
    ref_path = os.path.join(ref_dir, f"robot{agent_index}.json")
    with open(ref_path, "w") as f:
        json.dump(ref_measurements, f, indent=4)

    print(
        f"Created config/robot{agent_index}.json and ref-measurements/robot{agent_index}.json "
        f"using agent{agent_index} (RPC: {eth_node_url})."
    )

def create_secaas_config(config_dir: str):
    # MUST read keys from validator1
    validator_dir = os.path.join(NODES_DIR, "validator1")
    eth_node_url = pick_random_eth_node_url()
    creds = read_credentials_from_dir(validator_dir, eth_node_url)

    secaas_data = {
        "name": "secaas",
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],       # random validator URL
        "contract_address": contract_data["address"],
        "contract_abi": CONTRACT_ABI,
    }

    os.makedirs(config_dir, exist_ok=True)
    filepath = os.path.join(config_dir, "secaas.json")
    with open(filepath, "w") as f:
        json.dump(secaas_data, f, indent=4)

    print(f"Created config/secaas.json using validator1 (RPC: {eth_node_url}).")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create agent configs and ref-measurements from Besu quickstart files.")
    parser.add_argument("--num-agents", type=int, required=True, help="Number of agents to create")
    parser.add_argument("--config-output", default="./config", help="Output directory for normal config JSON files")
    parser.add_argument("--ref-output", default="./ref-measurements", help="Output directory for ref-measurements JSON files")
    args = parser.parse_args()

    os.makedirs(args.config_output, exist_ok=True)
    os.makedirs(args.ref_output, exist_ok=True)

    # Create secaas normal config
    create_secaas_config(args.config_output)

    # Create robots: normal config + ref-measurements
    for i in range(1, args.num_agents + 1):
        create_agent_files(i, args.config_output, args.ref_output)

    print(
        f"Successfully created {args.num_agents} robot configs in {args.config_output} "
        f"and ref-measurements in {args.ref_output}."
    )
