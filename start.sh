#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

# --- Paths ---
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
SECAAS_URL="http://localhost:8000"

# One Compose project name for ALL stacks
PROJECT_NAME="ros2-b-mutra"

PASSWORD="netcom;"

# Multi-host constants
LOCAL_LIMIT=8
REMOTE_HOST="10.5.1.21"
REMOTE_USER="desire6g"
REMOTE_DIR="~/ros2-b-mutra"

# local machine's own IP reachable from the remote host (used so remote sidecars can
# resolve host.docker.internal → local machine and reach the Besu validators).
LOCAL_IP="10.5.99.99"

# Ignore orphan warnings when using multiple compose files for the same project
export COMPOSE_IGNORE_ORPHANS=1

run_remote() {
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$REMOTE_USER@$REMOTE_HOST" "$@"
}

remote_compose_up() {
  local compose_file="$1"
  local retries=3
  local attempt
  for ((attempt = 1; attempt <= retries; attempt++)); do
    if run_remote "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME --project-directory $REMOTE_DIR -f $compose_file up -d"; then
      return 0
    fi
    if (( attempt < retries )); then
      echo "⚠️  Remote compose failed (attempt $attempt/$retries), retrying in 5s..."
      sleep 5
    fi
  done
  return 1
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [--robots N] [--auto] [--export] [--startup] [--wait-tx]

Options:
  --robots N  Number of robots to deploy (default: 4, max: 100).
              Regenerates docker-compose files before starting.
              N > $LOCAL_LIMIT activates multi-host mode (local machine + remote host).
  --auto      Set AUTO_START=TRUE
  --export    Set EXPORT_RESULTS=TRUE
  --wait-tx   Set WAIT_FOR_TX_CONFIRMATIONS=TRUE
  --startup   Set ONE_SHOT=TRUE
  -h|--help   Show this help

EOF
}


# Default values
N_ROBOTS_VAL="4"
AUTO_START_VAL="FALSE"
EXPORT_RESULTS_VAL="FALSE"
WAIT_TX_VAL="FALSE"
ONE_SHOT_VAL="FALSE"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)
      N_ROBOTS_VAL="$2"
      if ! [[ "$N_ROBOTS_VAL" =~ ^[1-9][0-9]*$ ]] || (( N_ROBOTS_VAL < 1 || N_ROBOTS_VAL > 100 )); then
        echo "❌ --robots must be an integer between 1 and 100 (got '$N_ROBOTS_VAL')"
        exit 1
      fi
      shift 2 ;;
    --auto)    AUTO_START_VAL="TRUE"; shift ;;
    --export)  EXPORT_RESULTS_VAL="TRUE"; shift ;;
    --wait-tx) WAIT_TX_VAL="TRUE"; shift ;;
    --startup) ONE_SHOT_VAL="TRUE"; shift ;;
    -h|--help) usage; exit 0 ;;
    *) echo "❌ Unknown arg: $1"; echo; usage; exit 1 ;;
  esac
done

# --- Ensure .env has desired values ---
upsert_env () {
  local key="$1"
  local val="$2"
  if grep -qE "^[[:space:]]*$key=" "$ENV_FILE"; then
    sed -i "s|^[[:space:]]*$key=.*|$key=$val|g" "$ENV_FILE"
  else
    echo "$key=$val" >> "$ENV_FILE"
  fi
}

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Creating missing .env at $ENV_FILE"
  touch "$ENV_FILE"
fi

upsert_env "EXPORT_RESULTS"           "$EXPORT_RESULTS_VAL"
upsert_env "AUTO_START"               "$AUTO_START_VAL"
upsert_env "WAIT_FOR_TX_CONFIRMATIONS" "$WAIT_TX_VAL"
upsert_env "ONE_SHOT"                 "$ONE_SHOT_VAL"
upsert_env "N_ROBOTS"                 "$N_ROBOTS_VAL"
upsert_env "COMPOSE_PROJECT_NAME"     "$PROJECT_NAME"

echo "✅ .env updated (Robots: $N_ROBOTS_VAL, Auto: $AUTO_START_VAL, Export: $EXPORT_RESULTS_VAL, Wait Tx: $WAIT_TX_VAL, Startup: $ONE_SHOT_VAL)"
echo

# Regenerate compose files
echo "🔧 Generating compose files for $N_ROBOTS_VAL robot(s)..."
python3 "$SCRIPT_DIR/generate_compose.py" --robots "$N_ROBOTS_VAL" --blockchain-host "$LOCAL_IP"
echo

# Reset EventWatcher checkpoints
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting EventWatcher checkpoints..."
  ( cd "$CHECKPOINT_DIR" && echo "$PASSWORD" | sudo -S bash "./$(basename "$RESET_SCRIPT")" )
  echo
else
  echo "⚠️  Skipping reset (missing $RESET_SCRIPT or directory $CHECKPOINT_DIR)"
  echo
fi

# ── Multi-host pre-flight ─────────────────────────────────────────────────────
if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  echo "🌐 Multi-host mode: local machine (robots 1–$LOCAL_LIMIT) + remote host ($REMOTE_HOST) (robots $((LOCAL_LIMIT+1))–$N_ROBOTS_VAL)"

  # Sync .env, generated compose files, and dummy configs to remote host
  echo "📤 Syncing .env, generated/, and config-dummy/ to remote host..."
  scp "$ENV_FILE" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR/.env"
  rsync -av "$SCRIPT_DIR/generated/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR/generated/"
  rsync -av "$SCRIPT_DIR/config-dummy/" "$REMOTE_USER@$REMOTE_HOST:$REMOTE_DIR/config-dummy/"
  echo
fi

# 1) Initialize the blockchain network
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./run.sh

# 2) Sleep 10
sleep 10

# 3) Deploy the smart contract
cd "$SCRIPT_DIR"
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337

# 4) Start robot containers
if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  echo "🤖 Starting robots (local: 1–$LOCAL_LIMIT)..."
  docker compose -p "$PROJECT_NAME" --project-directory "$SCRIPT_DIR" \
    -f "$SCRIPT_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-local.yml" up -d

  echo "🤖 Starting robots (remote: $((LOCAL_LIMIT + 1))–$N_ROBOTS_VAL on remote host)..."
  if ! remote_compose_up "$REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote.yml"; then
    echo "❌ Failed to start remote robots."
    exit 1
  fi
else
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" up -d
fi

# 5) Start docker-compose.secaas.yml (always on local machine)
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d


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

# 6) Wait and Sync
if wait_for_secaas; then
  echo "🔄 Synchronizing agent signatures to PostgreSQL..."
  curl -X POST "$SECAAS_URL/sync-agents?n_robots=$N_ROBOTS_VAL" -H "Content-Type: application/json" | jq
  echo "✅ Sync complete."
else
  echo "⚠️  Skipping agent sync because SECaaS is not responding."
  exit 1
fi

# 7) Start docker-compose.monitoring.yml (local machine always; remote host for multi-host stats coverage)
docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  echo "📊 Starting monitoring on remote host..."
  run_remote "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME \
    -f $REMOTE_DIR/docker-compose.monitoring.yml up -d"
fi

# 8) Start attestation sidecars
if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  echo "🛡️  Starting sidecars (local: 1–$LOCAL_LIMIT)..."
  docker compose -p "$PROJECT_NAME" --project-directory "$SCRIPT_DIR" \
    -f "$SCRIPT_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-local.yml" up -d

  echo "🛡️  Starting sidecars (remote: $((LOCAL_LIMIT + 1))–$N_ROBOTS_VAL on remote host)..."
  if ! remote_compose_up "$REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote.yml"; then
    echo "❌ Failed to start remote sidecars."
    exit 1
  fi
else
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" up -d
fi

echo "🎉 All done."
