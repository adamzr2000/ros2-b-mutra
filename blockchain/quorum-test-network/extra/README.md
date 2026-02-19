# Extra / Helper Scripts

This directory contains tools to generate cryptographic keys for Besu/Quorum nodes and automatically update the genesis configuration.

## Generate Credentials & Update Genesis

Run the main script to create keys and update [QBFTgenesis.json](../config/besu/QBFTgenesis.json)

```bash
# Default (20 agents)
./generate_agent_credentials.sh

# Custom number (e.g., 5 agents)
./generate_agent_credentials.sh -n 5
```

## Files

- `generate_agent_credentials.sh`: Main script. Creates directories in `../config/nodes/`, generates keys, and triggers the genesis update.

- `generate_node_details.js`: Helper script. Generates the actual node keys (P2P identity) and account keystores (signing wallet).

- `../config/update_qbft_genesis_agent_credentials.py`: Python script. Updates the alloc section of `QBFTgenesis.json` to fund the new agents.

## Output

Credentials are saved to: `../config/nodes/agentX/`

- nodekey, nodekey.pub: P2P Network Identity.

- accountKeystore, accountPassword: Transaction Signing Wallet.

- address: Ethereum Address.