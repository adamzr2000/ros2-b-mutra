#!/bin/bash

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
  echo "Error: --node-ip, --ws-port, and --http-port must be provided."
  echo "Usage: $0 --node-ip <IP> --ws-port <port> --http-port <port>"
  exit 1
fi

# Run truffle migrate command and capture the output
output=$(truffle migrate --network dlt_network)
# output_http=$(truffle migrate --network dlt_network_http)


# Print the output
echo "$output"

# Extract the contract address using grep and awk
contract_address=$(echo "$output" | grep "contract address:" | awk '{print $4}')

# Export both endpoints for config
eth_node_ws="ws://$node_ip:$ws_port"
eth_node_http="http://$node_ip:$http_port"

# Save the contract address and endpoints in the ../code/ directory
echo "CONTRACT_ADDRESS=$contract_address" > ./smart-contract.env
echo "WS_ENDPOINT=$eth_node_ws" >> ./smart-contract.env
echo "HTTP_ENDPOINT=$eth_node_http" >> ./smart-contract.env
echo "Contract Address saved in smart-contract.env file: $contract_address"
echo "WS Endpoint saved in smart-contract.env file: $eth_node_ws"
echo "HTTP Endpoint saved in smart-contract.env file: $eth_node_http"