#!/bin/bash

protocol="ws"  # default

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
    *)
      echo "Unknown option: $1"
      echo "Usage: $0 --node-ip <IP> --port <PORT> [--protocol ws|http]"
      exit 1
      ;;
  esac
done

if [[ -z "${node_ip:-}" || -z "${port:-}" ]]; then
  echo "Error: --node-ip and --port are required."
  echo "Usage: $0 --node-ip <IP> --port <PORT> [--protocol ws|http]"
  exit 1
fi

# Determine Truffle network and base URL
if [[ "$protocol" == "http" ]]; then
  echo "🔗 Deploying via HTTP to http://$node_ip:$port"
  output=$(truffle migrate --network geth_network_http)
  endpoint="http://$node_ip:$port"
elif [[ "$protocol" == "ws" ]]; then
  echo "🔗 Deploying via WebSocket to ws://$node_ip:$port"
  output=$(truffle migrate --network geth_network_ws)
  endpoint="ws://$node_ip:$port"
else
  echo "❌ Invalid protocol: $protocol (must be 'ws' or 'http')"
  exit 1
fi

# Extract contract address
contract_address=$(echo "$output" | grep "contract address:" | awk '{print $4}')

# Export env vars
echo "CONTRACT_ADDRESS=$contract_address" > ./smart-contract.env
echo "WS_ENDPOINT=ws://$node_ip:$port"       >> ./smart-contract.env
echo "HTTP_ENDPOINT=http://$node_ip:$port"  >> ./smart-contract.env

# Output summary
echo "✅ Contract deployed at: $contract_address"
echo "📄 Written to smart-contract.env"
