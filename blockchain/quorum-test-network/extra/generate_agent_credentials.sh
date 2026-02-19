#!/bin/bash
set -e

# Default number of agents
N=20

# Parse command line arguments for -n
while getopts "n:" opt; do
  case $opt in
    n) N="$OPTARG"
    ;;
    \?) echo "Invalid option -$OPTARG" >&2
    exit 1
    ;;
  esac
done

# Location of nodes config (relative to this script in 'extra/')
BASE_DIR="../config/nodes"

# Password for accounts (you can change if needed)
PASSWORD="password"

echo "--------------------------------------------------------"
echo ">>> Generating validator and node keys for $N agents"
echo "--------------------------------------------------------"

# Loop from agent1 to agentN
for i in $(seq 1 $N); do
    NODE_DIR="$BASE_DIR/agent$i"

    echo ">>> Processing $NODE_DIR"

    # Make directory if it does not exist
    mkdir -p "$NODE_DIR"

    # Go into the agent directory
    pushd "$NODE_DIR" > /dev/null

    # Clean up any old files to avoid overwrites
    rm -f accountKeystore accountPassword accountPrivateKey address nodekey nodekey.pub

    # Generate keys and account
    # We are deep inside ../config/nodes/agentX, so we reach back to extra/ for the JS
    node ../../../extra/generate_node_details.js --password "$PASSWORD"

    popd > /dev/null
done

echo "✅ Finished creating keys for agent1 through agent$N"

echo "--------------------------------------------------------"
echo ">>> Updating Genesis configuration (QBFTgenesis.json)"
echo "--------------------------------------------------------"

# Switch context to the 'config' directory so the Python script's relative paths work
pushd ../config > /dev/null

# Execute the python script with the number of agents
if [ -f "update_qbft_genesis_agent_credentials.py" ]; then
    python3 update_qbft_genesis_agent_credentials.py "$N"
else
    echo "❌ Error: Could not find update_qbft_genesis_agent_credentials.py in ../config/"
    exit 1
fi

# Return to original directory
popd > /dev/null

echo "✅ All operations complete."