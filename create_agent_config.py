import json
import os
import hashlib
import argparse
import random

BESU_BASE = "blockchain/quorum-test-network"
NODES_DIR = os.path.join(BESU_BASE, "config", "nodes")

# Validator RPC ports exposed on the Docker host (validator1→21001 … validator4→21004)
_VALIDATOR_PORTS = [21001, 21002, 21003, 21004]

_DEFAULT_CONTRACT = "AttestationManagerRR"  # ← change this to switch contracts


def pick_random_eth_node_url(blockchain_host: str) -> str:
    port = random.choice(_VALIDATOR_PORTS)
    return f"http://{blockchain_host}:{port}"


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


def create_agent_files(agent_index: int, config_dir: str, ref_dir, blockchain_host: str,
                       cmd_name: str, text_section_size: int, offset: int, text_section_prefix: str,
                       contract_address: str, contract_abi: list):
    # robot i -> agent i
    agent_dir = os.path.join(NODES_DIR, f"agent{agent_index}")
    eth_node_url = pick_random_eth_node_url(blockchain_host)
    creds = read_credentials_from_dir(agent_dir, eth_node_url)

    # 1) Normal config -> ./config/robot{i}.json
    robot_config = {
        "name": f"robot{agent_index}",
        "cmd_name": cmd_name,
        "text_section_size": text_section_size,
        "offset": offset,
        "text_section_prefix": text_section_prefix,
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],
        "contract_address": contract_address,
        "contract_abi": contract_abi,
    }

    os.makedirs(config_dir, exist_ok=True)
    config_path = os.path.join(config_dir, f"robot{agent_index}.json")
    with open(config_path, "w") as f:
        json.dump(robot_config, f, indent=4)

    if ref_dir is None:
        print(f"Created {config_path} using agent{agent_index} (RPC: {eth_node_url}).")
        return

    # 2) Ref-measurements -> ./ref-measurements/robot{i}.json (placeholder, replaced by bootstrap)
    fake_hash = generate_fake_hash(f"robot{agent_index}")
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
        f"Created {config_path} and {ref_path} "
        f"using agent{agent_index} (RPC: {eth_node_url})."
    )


def create_secaas_config(config_dir: str, blockchain_host: str,
                         contract_address: str, contract_abi: list):
    # MUST read keys from validator1
    validator_dir = os.path.join(NODES_DIR, "validator1")
    eth_node_url = pick_random_eth_node_url(blockchain_host)
    creds = read_credentials_from_dir(validator_dir, eth_node_url)

    secaas_data = {
        "name": "secaas",
        "eth_address": creds["eth_address"],
        "private_key": creds["private_key"],
        "eth_node_url": creds["eth_node_url"],       # random validator URL
        "contract_address": contract_address,
        "contract_abi": contract_abi,
    }

    os.makedirs(config_dir, exist_ok=True)
    filepath = os.path.join(config_dir, "secaas.json")
    with open(filepath, "w") as f:
        json.dump(secaas_data, f, indent=4)

    print(f"Created {filepath} using validator1 (RPC: {eth_node_url}).")


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Create agent configs and ref-measurements from Besu quickstart files.")
    parser.add_argument("--num-agents", type=int, required=True, help="Number of agents to create")
    parser.add_argument("--contract", default=_DEFAULT_CONTRACT,
                        help=f"Smart contract name to use (default: {_DEFAULT_CONTRACT})")
    parser.add_argument("--config-output", default="./config", help="Output directory for normal config JSON files")
    parser.add_argument("--ref-output", default=None,
                        help="Output directory for ref-measurements JSON files. Omit to skip writing ref-measurements.")
    parser.add_argument("--blockchain-host", default="host.docker.internal",
                        help="Hostname/IP for Besu RPC endpoints (default: host.docker.internal)")
    parser.add_argument("--secaas-host", default=None,
                        help="Hostname/IP for Besu RPC used by SECaaS config; falls back to --blockchain-host if omitted")
    parser.add_argument("--cmd-name", default="robot_state_publisher",
                        help="Process name to attest (default: robot_state_publisher)")
    parser.add_argument("--text-section-size", type=int, default=42223,
                        help="ELF .text section size in bytes (default: 42223 for robot_state_publisher)")
    parser.add_argument("--offset", type=int, default=0,
                        help="ELF .text section offset (default: 0)")
    parser.add_argument("--text-section-prefix", default="",
                        help="ELF .text section prefix (default: empty)")
    args = parser.parse_args()

    # Load contract deployment info and ABI based on selected contract
    contract_name = args.contract
    with open(f"smart-contracts/deployments/besu-{contract_name}.json") as f:
        contract_data = json.load(f)

    abi_path = f"smart-contracts/artifacts/contracts/{contract_name}.sol/{contract_name}.json"
    with open(abi_path) as f:
        artifact = json.load(f)
    if "abi" not in artifact:
        raise KeyError(f"ABI not found in artifact: {abi_path}")
    contract_abi = artifact["abi"]

    print(f"ℹ️  Using contract: {contract_name}  ({contract_data['address']})")

    os.makedirs(args.config_output, exist_ok=True)

    secaas_host = args.secaas_host or args.blockchain_host
    create_secaas_config(args.config_output, secaas_host,
                         contract_data["address"], contract_abi)

    for i in range(1, args.num_agents + 1):
        create_agent_files(i, args.config_output, args.ref_output, args.blockchain_host,
                           args.cmd_name, args.text_section_size, args.offset, args.text_section_prefix,
                           contract_data["address"], contract_abi)

    msg = f"Successfully created {args.num_agents} robot configs in {args.config_output}"
    if args.ref_output:
        msg += f" and ref-measurements in {args.ref_output}"
    print(msg + ".")
