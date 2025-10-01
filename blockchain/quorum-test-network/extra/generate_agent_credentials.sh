#!/bin/bash
set -e

# Location of nodes config
BASE_DIR="../config/nodes"

# Password for accounts (you can change if needed)
PASSWORD="password"

echo ">>> Generating validator and node keys for agent1 through agent20"
# Loop from agent1 to agent10
for i in $(seq 1 20); do
    NODE_DIR="$BASE_DIR/agent$i"

    echo ">>> Creating $NODE_DIR"

    # Make directory if it does not exist
    mkdir -p "$NODE_DIR"

    # Go into the validator directory
    pushd "$NODE_DIR" > /dev/null

    # Clean up any old files to avoid overwrites
    rm -f accountKeystore accountPassword accountPrivateKey address nodekey nodekey.pub

    # Generate keys and account
    node ../../../extra/generate_node_details.js --password "$PASSWORD"

    popd > /dev/null
done

echo "✅ Finished creating agent1 through agent20"
