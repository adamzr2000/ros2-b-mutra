#!/bin/bash
set -e

# 1. Initialize the blockchain network
cd blockchain/quorum-test-network
./run.sh

sleep 5

# 2. Deploy the smart contract
cd ../..
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337

# 3. Start containers
docker compose up -d

sleep 10

# 4. Start docker stats data collection
curl -X POST localhost:6666/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{"containers": ["secaas-sidecar","robot1-sidecar","robot2-sidecar", "robot3-sidecar"],
  "interval":1.0,"csv_dir":"/experiments/data/docker-stats/test","stdout":true}' | jq
  
# Map sidecars (names) to ports — edit to match yours
declare -A SIDECAR_PORTS=(
  [secaas]=8080
  [robot1]=8081
  [robot2]=8082
  [robot3]=8083
)

base_url="http://localhost"

# Start attestation on all sidecars
for name in "${!SIDECAR_PORTS[@]}"; do
    p=${SIDECAR_PORTS[$name]}
    echo "▶️  Starting attestation on $name (port $p)..."
    curl -X POST "$base_url:$p/start" | jq
  done