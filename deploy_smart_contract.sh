#!/bin/bash

# Initialize variables
node_ip=""
ws_port=""
http_port=""

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
    --http-port)
      http_port="$2"
      shift 2
      ;;
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 --node-ip <IP> --ws-port <port> --http-port <port>"
      exit 1
      ;;
  esac
done

# Ensure required arguments are provided
if [[ -z "$node_ip" || -z "$ws_port" || -z "$http_port" ]]; then
  echo "Error: Both --node-ip, --ws-port and --http-port must be provided."
  echo "Usage: $0 --node-ip <IP> --ws-port <port> --http-port <port>"
  exit 1
fi

# Export both endpoints for config
eth_node_ws="ws://$node_ip:$ws_port"
eth_node_http="http://$node_ip:$http_port"
# ... (continue with the rest of your deployment logic, ensuring both endpoints are written to config files)

echo "Starting Truffle container with:"
echo " - Node IP: $node_ip"
echo " - WebSocket Port: $ws_port"

# Construct the start command for deploying the smart contract
# START_CMD="./deploy.sh"
START_CMD="./deploy.sh --node-ip $node_ip --ws-port $ws_port --http-port $http_port"


# Start a Docker container with the specified configurations
# docker run \
#   -it \
#   --rm \
#   --name truffle \
#   --hostname truffle \
#   --network dlt_network \
#   -v "$(pwd)/smart-contracts":/smart-contracts \
#   -e NODE_IP="$node_ip" \
#   -e WS_PORT="$ws_port" \
#   truffle:latest \
#   $START_CMD

  docker run \
  -it \
  --rm \
  --name truffle \
  --hostname truffle \
  --network dlt_network \
  -v "$(pwd)/smart-contracts":/smart-contracts \
  -e NODE_IP="$node_ip" \
  -e WS_PORT="$ws_port" \
  -e HTTP_PORT="$http_port" \
  truffle:latest \
  bash -c "cd /smart-contracts && $START_CMD"