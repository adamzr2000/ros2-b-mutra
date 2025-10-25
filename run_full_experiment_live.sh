#!/bin/bash
set -e

compose_pid=""

cleanup() {
    echo -e "\n[!] Caught Ctrl+C or Compose exited, stopping full experiment..."
    ./stop_full_experiment.sh
    exit 1
}

# Trap SIGINT and SIGTERM
trap cleanup SIGINT SIGTERM

# 1. Initialize the blockchain network
cd blockchain/quorum-test-network
./run.sh

sleep 5

# 2. Deploy the smart contract
cd ../..
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337

# 3. Start Docker Compose in background
docker compose up &
compose_pid=$!

# Wait for Compose to finish (naturally or via Ctrl+C)
wait $compose_pid

# Always call cleanup to stop geth if Compose exits cleanly
cleanup
