#!/bin/bash

# Initialize variables
node_ip=""
port=""
protocol="ws"      # default
network_id="2024"  # default

# Parse arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --node-ip)
      node_ip="$2"
      shift 2
      ;;
    --port)
      port="$2"
      shift 2
      ;;
    --protocol)
      protocol="$2"
      shift 2
      ;;
    --network-id)
      network_id="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 --node-ip <IP> --port <PORT> [--protocol ws|http] [--network-id <ID>]"
      exit 1
      ;;
  esac
done

# Check required args
if [[ -z "$node_ip" || -z "$port" ]]; then
  echo "Error: --node-ip and --port are required."
  echo "Usage: $0 --node-ip <IP> --port <PORT> [--protocol ws|http] [--network-id <ID>]"
  exit 1
fi

echo "🚀 Starting Truffle container with:"
echo " - Node IP    : $node_ip"
echo " - Port       : $port"
echo " - Protocol   : $protocol"
echo " - Network ID : $network_id"

# Construct the command for the container
START_CMD="./deploy.sh --node-ip $node_ip --port $port --protocol $protocol"

docker run \
  -it \
  --rm \
  --name truffle \
  --hostname truffle \
  --network blockchain_network \
  -v "$(pwd)/smart-contracts":/smart-contracts \
  -e NODE_IP="$node_ip" \
  -e PORT="$port" \
  -e PROTOCOL="$protocol" \
  -e NETWORK_ID="$network_id" \
  truffle:latest \
  bash -c "cd /smart-contracts && $START_CMD"
