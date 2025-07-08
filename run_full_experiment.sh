#!/bin/bash
set -e

# 1. Initialize the blockchain network
cd blockchain-network/geth-poa
./start_geth_net.sh

sleep 3

# 2. Deploy the smart contract
cd ../..
./deploy_smart_contract.sh --node-ip 10.0.1.1 --port 3334 --protocol http

sleep 3

# 3. Start the experimental scenario (docker compose)
docker compose up -d