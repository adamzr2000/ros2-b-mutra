#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# --- Paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"

# --- Ensure .env has desired values BEFORE anything else ---
upsert_env () {
  local key="$1"
  local val="$2"
  if grep -qE "^[[:space:]]*$key=" "$ENV_FILE"; then
    # replace existing line
    sed -i "s|^[[:space:]]*$key=.*|$key=$val|g" "$ENV_FILE"
  else
    # append new key
    echo "$key=$val" >> "$ENV_FILE"
  fi
}

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Creating missing .env at $ENV_FILE"
  touch "$ENV_FILE"
fi

# Set required flags
upsert_env "EXPORT_RESULTS" "FALSE"
upsert_env "AUTO_START" "TRUE"

echo "✅ .env updated:"
grep -E '^(EXPORT_RESULTS|AUTO_START)=' "$ENV_FILE" || true
echo

# Reset EventWatcher checkpoints (seen_keys/from_block) before starting experiment
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting EventWatcher checkpoints..."
  ( cd "$CHECKPOINT_DIR" && bash "./$(basename "$RESET_SCRIPT")" )
  echo
else
  echo "⚠️  Skipping reset (missing $RESET_SCRIPT or directory $CHECKPOINT_DIR)"
  echo
fi

# Initialize the blockchain network
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./run.sh

sleep 10

# Deploy the smart contract
cd "$SCRIPT_DIR"
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337

# Start containers (docker compose will read the updated .env)
docker compose up -d

echo "🎉 All done."
