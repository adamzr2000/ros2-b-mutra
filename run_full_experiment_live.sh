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

# Start blockchain and contract
cd blockchain-network/geth-poa
./start_geth_net.sh
cd ../..
./deploy_smart_contract.sh --node-ip 10.0.1.1 --port 3334 --protocol http

# Start Docker Compose in background
docker compose up &
compose_pid=$!

# Wait for Compose to finish (naturally or via Ctrl+C)
wait $compose_pid

# Always call cleanup to stop geth if Compose exits cleanly
cleanup
