#!/usr/bin/env bash
set -euo pipefail

# Initialize
private_key=""
rpc_url=""
chain_id=""

# Default path if private_key not passed
DEFAULT_KEY_PATH="blockchain/quorum-test-network/config/nodes/validator1/accountPrivateKey"

usage() {
  echo "Usage: $0 [--private_key <hexkey>] --rpc_url <url> --chain_id <id>"
  echo "       If --private_key is not given, will try to read from:"
  echo "       $DEFAULT_KEY_PATH"
  exit 1
}

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --private_key) private_key="${2:-}"; shift 2 ;;
    --rpc_url)     rpc_url="${2:-}";     shift 2 ;;
    --chain_id)    chain_id="${2:-}";    shift 2 ;;
    -h|--help)     usage ;;
    *) echo "Unknown option: $1"; usage ;;
  esac
done

# If private_key not provided, try default file
if [[ -z "$private_key" ]]; then
  if [[ -f "$DEFAULT_KEY_PATH" ]]; then
    private_key=$(<"$DEFAULT_KEY_PATH")
    echo "ℹ️  Loaded private key from $DEFAULT_KEY_PATH"
  else
    echo "❌ Error: no --private_key passed and default key file not found: $DEFAULT_KEY_PATH"
    exit 1
  fi
fi

# Validate required args
if [[ -z "$rpc_url" || -z "$chain_id" ]]; then
  echo "❌ Error: --rpc_url and --chain_id are required."
  usage
fi

echo "🚀 Deploying Federation contract"
echo "Private Key: [HIDDEN]"
echo "RPC URL    : $rpc_url"
echo "Chain ID   : $chain_id"

# Run in Docker
docker run -it --rm --name hardhat \
  -v "$(pwd)/smart-contracts/scripts":/smart-contracts/scripts \
  -v "$(pwd)/smart-contracts/deployments":/smart-contracts/deployments \
  -v "$(pwd)/smart-contracts/contracts":/smart-contracts/contracts \
  -v "$(pwd)/smart-contracts/test":/smart-contracts/test \
  -v "$(pwd)/smart-contracts/artifacts":/smart-contracts/artifacts \
  -v "$(pwd)/smart-contracts/hardhat.config.js":/smart-contracts/hardhat.config.js \
  -e PRIVATE_KEY="$private_key" \
  -e RPC_URL="$rpc_url" \
  -e CHAIN_ID="$chain_id" \
  hardhat:latest \
  bash -lc "npx hardhat run scripts/deploy_contract.js --network besu"