#!/bin/bash
set -e

# 1. Stop attestation on all sidecars
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
    echo "▶️  Stopping attestation on $name (port $p)..."
    curl -X POST "$base_url:$p/stop" | jq
  done

# 2. Stop docker stats data collection
curl -X POST "http://localhost:6666/monitor/stop" | jq

# 3. Stop experimental scenario
PASSWORD="netcom;"
echo "$PASSWORD" | sudo -S docker compose down

# 4. Remove blockchain network 
cd blockchain/quorum-test-network
./remove.sh