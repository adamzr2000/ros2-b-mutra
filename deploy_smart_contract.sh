#!/bin/bash

# Initialize variables
node_ip=""
ws_port=""

# Parse command-line arguments
while [[ $# -gt 0 ]]; do
  case "$1" in
    --node-ip)
      node_ip="$2"
      shift 2
      ;;
    --ws-port)
      ws_port="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 --node-ip <IP> --ws-port <port>"
      exit 1
      ;;
  esac
done

# Ensure required arguments are provided
if [[ -z "$node_ip" || -z "$ws_port" ]]; then
  echo "Error: Both --node-ip and --ws-port must be provided."
  echo "Usage: $0 --node-ip <IP> --ws-port <port>"
  exit 1
fi

echo "Starting Truffle container with:"
echo " - Node IP: $node_ip"
echo " - WebSocket Port: $ws_port"

# Construct the start command for deploying the smart contract
START_CMD="./deploy.sh"

# Start a Docker container with the specified configurations
docker run \
  -it \
  --rm \
  --name truffle \
  --hostname truffle \
  --network dlt_network \
  -v "$(pwd)/smart-contracts":/smart-contracts \
  -e NODE_IP="$node_ip" \
  -e WS_PORT="$ws_port" \
  truffle:latest \
  $START_CMD