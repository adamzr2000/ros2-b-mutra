#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# --- Paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
SECAAS_URL="http://localhost:8000"

# One Compose project name for ALL stacks
PROJECT_NAME="ros2-b-mutra"

# Ignore orphan warnings when using multiple compose files for the same project
export COMPOSE_IGNORE_ORPHANS=1

usage() {
  cat <<EOF
Usage: $(basename "$0") [--auto|--no-auto] [--export|--no-export]

Defaults:
  --no-auto     (AUTO_START=FALSE)
  --no-export   (EXPORT_RESULTS=FALSE)

Options:
  --auto        Set AUTO_START=TRUE
  --no-auto     Set AUTO_START=FALSE
  --export      Set EXPORT_RESULTS=TRUE
  --no-export   Set EXPORT_RESULTS=FALSE

Examples:
  ./start.sh
  ./start.sh --auto
  ./start.sh --export
  ./start.sh --auto --export
EOF
}

# Defaults (requested): no-auto + no-export
AUTO_START_VAL="FALSE"
EXPORT_RESULTS_VAL="FALSE"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --auto)      AUTO_START_VAL="TRUE"; shift ;;
    --no-auto)   AUTO_START_VAL="FALSE"; shift ;;
    --export)    EXPORT_RESULTS_VAL="TRUE"; shift ;;
    --no-export) EXPORT_RESULTS_VAL="FALSE"; shift ;;
    -h|--help)   usage; exit 0 ;;
    *) echo "❌ Unknown arg: $1"; echo; usage; exit 1 ;;
  esac
done

# --- Ensure .env has desired values BEFORE anything else ---
upsert_env () {
  local key="$1"
  local val="$2"
  if grep -qE "^[[:space:]]*$key=" "$ENV_FILE"; then
    sed -i "s|^[[:space:]]*$key=.*|$key=$val|g" "$ENV_FILE"
  else
    echo "$key=$val" >> "$ENV_FILE"
  fi
}

wait_for_secaas () {
  echo "⏳ Waiting for SECaaS API to be ready at $SECAAS_URL..."
  local max_attempts=30
  local attempt=1

  while [ $attempt -le $max_attempts ]; do
    if curl -s "$SECAAS_URL/" | grep -q "SECaaS running"; then
      echo "✅ SECaaS is UP and running!"
      return 0
    fi
    echo "   (Attempt $attempt/$max_attempts) Still waiting..."
    sleep 2
    attempt=$((attempt + 1))
  done

  echo "❌ SECaaS failed to start in time."
  return 1
}

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Creating missing .env at $ENV_FILE"
  touch "$ENV_FILE"
fi

# Apply chosen flags
upsert_env "EXPORT_RESULTS" "$EXPORT_RESULTS_VAL"
upsert_env "AUTO_START" "$AUTO_START_VAL"
upsert_env "COMPOSE_PROJECT_NAME" "$PROJECT_NAME"

echo "✅ .env updated:"
grep -E '^(EXPORT_RESULTS|AUTO_START|COMPOSE_PROJECT_NAME)=' "$ENV_FILE" || true
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

# 1) Initialize the blockchain network
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./run.sh

# 2) sleep 10
sleep 10

# 3) Deploy the smart contract
cd "$SCRIPT_DIR"
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337

# 4) Start docker-compose.robots.yml
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" up -d

# 5) Start docker-compose.secaas.yml
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d

# 6) Wait and Sync
if wait_for_secaas; then
  echo "🔄 Synchronizing agent signatures to PostgreSQL..."
  curl -X POST "$SECAAS_URL/sync-agents" -H "Content-Type: application/json" | jq
  echo "✅ Sync complete."
else
  echo "⚠️  Skipping agent sync because SECaaS is not responding."
  exit 1
fi

# 7) Start docker-compose.monitoring.yml
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

# 8) Start docker-compose.attestation.yml
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" up -d

echo "🎉 All done."
