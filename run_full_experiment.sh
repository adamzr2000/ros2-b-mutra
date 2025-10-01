#!/bin/bash
set -e

# 1. Initialize the blockchain network
cd blockchain/quorum-test-network
./run.sh

sleep 5

# 2. Deploy the smart contract
cd ../..
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337

sleep 5

# 3. Start the experimental scenario (docker compose)
docker compose up -d