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

# Remote host constants
REMOTE1_HOST="10.5.1.21"
REMOTE1_USER="desire6g"
REMOTE1_LIMIT=64
REMOTE2_HOST=""          # placeholder — set when a second remote host is available
REMOTE2_USER="desire6g"
REMOTE2_LIMIT=128
REMOTE_DIR="~/ros2-b-mutra"

# Local machine's IP reachable from remote hosts (injected as host.docker.internal
# so remote sidecars can reach the Besu validators on d-mutra).
LOCAL_IP="10.5.99.99"

# Ignore orphan warnings when using multiple compose files for the same project
export COMPOSE_IGNORE_ORPHANS=1

run_remote() {
  local host="$1"
  local user="$2"
  shift 2
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$user@$host" "$@"
}

remote_compose_up() {
  local host="$1"
  local user="$2"
  local compose_file="$3"
  local retries=3
  local attempt
  for ((attempt = 1; attempt <= retries; attempt++)); do
    if run_remote "$host" "$user" \
        "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME --project-directory $REMOTE_DIR -f $compose_file up -d"; then
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
Usage: $(basename "$0") [--robots N] [--remote] [--auto] [--export] [--startup] [--wait-tx]
                        [--ssp N] [--cpu-limit X] [--contract standard|optimized] [--vrp N]

Options:
  --robots N     Number of robots to deploy (default: 4, max: $REMOTE2_LIMIT).
  --remote       Remote deployment mode: d-mutra hosts only Besu+SECaaS+monitoring;
                 all robots+sidecars run on remote host(s) (remote1 up to $REMOTE1_LIMIT robots,
                 remote2 for $((REMOTE1_LIMIT+1))–$REMOTE2_LIMIT robots).
                 Default (local mode): everything runs on d-mutra.
  --auto         Set AUTO_START=TRUE
  --export       Set EXPORT_RESULTS=TRUE
  --wait-tx      Set WAIT_FOR_TX_CONFIRMATIONS=TRUE
  --startup      Set ONE_SHOT=TRUE
  --ssp N        Attestation interval in seconds — sets ATTESTATION_INTERVAL_MS=N*1000 (default: 20)
  --cpu-limit X  Sidecar CPU limit fraction — sets CPU_LIMIT=X in .env and compose files (default: 0.4)
  --contract standard|optimized  Smart contract variant (default: optimized)
  --vrp N        Verifier Refreshing Period for optimized contract (default: 1)
  -h|--help      Show this help

EOF
}

# Default values
N_ROBOTS_VAL="4"
AUTO_START_VAL="FALSE"
EXPORT_RESULTS_VAL="FALSE"
WAIT_TX_VAL="FALSE"
ONE_SHOT_VAL="FALSE"
DEPLOY_MODE="local"
SSP_VAL="20"
CPU_LIMIT_VAL="0.4"
VARIANT_VAL="optimized"
CONTRACT_VAL="AttestationManagerOptimized"
VRP_VAL="1"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)
      N_ROBOTS_VAL="$2"
      if ! [[ "$N_ROBOTS_VAL" =~ ^[1-9][0-9]*$ ]] || (( N_ROBOTS_VAL < 1 || N_ROBOTS_VAL > REMOTE2_LIMIT )); then
        echo "❌ --robots must be an integer between 1 and $REMOTE2_LIMIT (got '$N_ROBOTS_VAL')"
        exit 1
      fi
      shift 2 ;;
    --remote)   DEPLOY_MODE="remote"; shift ;;
    --auto)     AUTO_START_VAL="TRUE"; shift ;;
    --export)   EXPORT_RESULTS_VAL="TRUE"; shift ;;
    --wait-tx)  WAIT_TX_VAL="TRUE"; shift ;;
    --startup)  ONE_SHOT_VAL="TRUE"; shift ;;
    --ssp)
      SSP_VAL="$2"
      if ! [[ "$SSP_VAL" =~ ^[1-9][0-9]*$ ]]; then
        echo "❌ --ssp must be a positive integer in seconds (got '$SSP_VAL')"
        exit 1
      fi
      shift 2 ;;
    --cpu-limit)
      CPU_LIMIT_VAL="$2"
      if ! [[ "$CPU_LIMIT_VAL" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "❌ --cpu-limit must be a positive number (got '$CPU_LIMIT_VAL')"
        exit 1
      fi
      shift 2 ;;
    --contract)
      VARIANT_VAL="$2"
      if [[ "$VARIANT_VAL" != "standard" && "$VARIANT_VAL" != "optimized" ]]; then
        echo "❌ --contract must be 'standard' or 'optimized' (got '$VARIANT_VAL')"
        exit 1
      fi
      shift 2 ;;
    --vrp)
      VRP_VAL="$2"
      if ! [[ "$VRP_VAL" =~ ^[1-9][0-9]*$ ]]; then
        echo "❌ --vrp must be a positive integer (got '$VRP_VAL')"
        exit 1
      fi
      shift 2 ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "❌ Unknown arg: $1"; echo; usage; exit 1 ;;
  esac
done

# Map user-facing variant to internal contract name
if [[ "$VARIANT_VAL" == "optimized" ]]; then
  CONTRACT_VAL="AttestationManagerOptimized"
else
  CONTRACT_VAL="AttestationManager"
fi

# Validate remote2 requirements before proceeding
if [[ "$DEPLOY_MODE" == "remote" ]] && (( N_ROBOTS_VAL > REMOTE1_LIMIT )) && [[ -z "$REMOTE2_HOST" ]]; then
  echo "❌ N=$N_ROBOTS_VAL > REMOTE1_LIMIT=$REMOTE1_LIMIT but REMOTE2_HOST is not configured in start.sh."
  exit 1
fi

# --- Ensure .env has desired values ---
upsert_env() {
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

upsert_env "EXPORT_RESULTS"            "$EXPORT_RESULTS_VAL"
upsert_env "AUTO_START"                "$AUTO_START_VAL"
upsert_env "WAIT_FOR_TX_CONFIRMATIONS" "$WAIT_TX_VAL"
upsert_env "ONE_SHOT"                  "$ONE_SHOT_VAL"
upsert_env "N_ROBOTS"                  "$N_ROBOTS_VAL"
upsert_env "COMPOSE_PROJECT_NAME"      "$PROJECT_NAME"
upsert_env "DEPLOY_MODE"               "$DEPLOY_MODE"
upsert_env "ATTESTATION_INTERVAL_MS"  "$((SSP_VAL * 1000))"
upsert_env "CPU_LIMIT"                 "$CPU_LIMIT_VAL"

echo "✅ .env updated (Robots: $N_ROBOTS_VAL, Mode: $DEPLOY_MODE, Contract: $CONTRACT_VAL, Auto: $AUTO_START_VAL, Export: $EXPORT_RESULTS_VAL, Startup: $ONE_SHOT_VAL, SSP: ${SSP_VAL}s, CPU: $CPU_LIMIT_VAL)"
echo

# Regenerate compose files for the selected mode
echo "🔧 Generating compose files for $N_ROBOTS_VAL robot(s) [mode: $DEPLOY_MODE]..."
python3 "$SCRIPT_DIR/generate_compose.py" --robots "$N_ROBOTS_VAL" --blockchain-host "$LOCAL_IP" --mode "$DEPLOY_MODE" --contract "$CONTRACT_VAL"
echo

# Reset local EventWatcher checkpoints (SECaaS always on d-mutra)
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting local EventWatcher checkpoints..."
  ( cd "$CHECKPOINT_DIR" && echo "$PASSWORD" | sudo -S bash "./$(basename "$RESET_SCRIPT")" )
  echo
else
  echo "⚠️  Skipping local checkpoint reset (missing $RESET_SCRIPT or $CHECKPOINT_DIR)"
  echo
fi

# ── Initialize the blockchain network (always on d-mutra) ─────────────────────
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./run.sh

sleep 10

# ── Deploy the smart contract ──────────────────────────────────────────────────
cd "$SCRIPT_DIR"
deploy_flags=(--rpc_url http://localhost:21001 --chain_id 1337 --contract "$CONTRACT_VAL")
[[ "$CONTRACT_VAL" == "AttestationManagerOptimized" ]] && deploy_flags+=(--vrp "$VRP_VAL")
./deploy_sc.sh "${deploy_flags[@]}"

# ── Generate agent config JSONs (config/ and config-dummy/) ───────────────────
# Blockchain host seen by sidecar containers differs per deploy mode.
if [[ "$DEPLOY_MODE" == "remote" ]]; then
  CFG_BLOCKCHAIN_HOST="$LOCAL_IP"
else
  CFG_BLOCKCHAIN_HOST="host.docker.internal"
fi

echo "🔧 Generating agent configs (contract: $CONTRACT_VAL, blockchain-host: $CFG_BLOCKCHAIN_HOST)..."

python3 "$SCRIPT_DIR/create_agent_config.py" \
  --num-agents 100 \
  --contract "$CONTRACT_VAL" \
  --config-output "$SCRIPT_DIR/config" \
  --ref-output "$SCRIPT_DIR/ref-measurements" \
  --blockchain-host "$CFG_BLOCKCHAIN_HOST" \
  --cmd-name robot_state_publisher \
  --text-section-size 42223 \
  --offset 0

python3 "$SCRIPT_DIR/create_agent_config.py" \
  --num-agents 100 \
  --contract "$CONTRACT_VAL" \
  --config-output "$SCRIPT_DIR/config-dummy" \
  --ref-output "$SCRIPT_DIR/ref-measurements-dummy" \
  --blockchain-host "$CFG_BLOCKCHAIN_HOST" \
  --cmd-name dummy_publisher \
  --text-section-size 175521 \
  --offset 5744

echo

# ── SECaaS helper (shared by both deployment branches) ────────────────────────
wait_for_secaas() {
  echo "⏳ Waiting for SECaaS API to be ready at $SECAAS_URL..."
  local max_attempts=30
  local attempt=1
  while (( attempt <= max_attempts )); do
    if curl -s "$SECAAS_URL/" | grep -q "SECaaS running"; then
      echo "✅ SECaaS is UP and running!"
      return 0
    fi
    echo "   (Attempt $attempt/$max_attempts) Still waiting..."
    sleep 2
    (( attempt++ ))
  done
  echo "❌ SECaaS failed to start in time."
  return 1
}

# ══════════════════════════════════════════════════════════════════════════════
if [[ "$DEPLOY_MODE" == "remote" ]]; then
# ── REMOTE MODE ───────────────────────────────────────────────────────────────
# d-mutra: Besu + SECaaS + monitoring only.
# All robots + sidecars run on remote host(s).
  echo "🌐 Remote mode: d-mutra hosts Besu+SECaaS+monitoring; robots+sidecars on remote host(s)"
  echo

  # Sync .env, generated/, config/, and config-dummy/ to remote1
  echo "📤 Syncing to remote1 ($REMOTE1_HOST)..."
  scp "$ENV_FILE" "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/.env"
  rsync -av "$SCRIPT_DIR/generated/"    "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/generated/"
  rsync -av "$SCRIPT_DIR/config/"       "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/config/"
  rsync -av "$SCRIPT_DIR/config-dummy/" "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/config-dummy/"
  echo

  # Sync to remote2 if N > REMOTE1_LIMIT (REMOTE2_HOST validated above)
  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "📤 Syncing to remote2 ($REMOTE2_HOST)..."
    scp "$ENV_FILE" "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/.env"
    rsync -av "$SCRIPT_DIR/generated/"    "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/generated/"
    rsync -av "$SCRIPT_DIR/config-dummy/" "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/config-dummy/"
    echo
  fi

  # Start SECaaS on d-mutra
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d

  if wait_for_secaas; then
    echo "🔄 Synchronizing agent signatures to PostgreSQL..."
    curl -X POST "$SECAAS_URL/sync-agents?n_robots=$N_ROBOTS_VAL" -H "Content-Type: application/json" | jq
    echo "✅ Sync complete."
  else
    exit 1
  fi

  # Start monitoring on d-mutra
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

  # Start robots on remote1
  echo "🤖 Starting robots on remote1 ($REMOTE1_HOST, robots 1–$(( N_ROBOTS_VAL < REMOTE1_LIMIT ? N_ROBOTS_VAL : REMOTE1_LIMIT )))..."
  if ! remote_compose_up "$REMOTE1_HOST" "$REMOTE1_USER" \
      "$REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote1.yml"; then
    echo "❌ Failed to start robots on remote1."
    exit 1
  fi

  # Start robots on remote2 if N > REMOTE1_LIMIT
  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "🤖 Starting robots on remote2 ($REMOTE2_HOST, robots $((REMOTE1_LIMIT+1))–$N_ROBOTS_VAL)..."
    if ! remote_compose_up "$REMOTE2_HOST" "$REMOTE2_USER" \
        "$REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote2.yml"; then
      echo "❌ Failed to start robots on remote2."
      exit 1
    fi
  fi

  # Start monitoring on remote1
  echo "📊 Starting monitoring on remote1..."
  run_remote "$REMOTE1_HOST" "$REMOTE1_USER" \
    "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME \
     -f $REMOTE_DIR/docker-compose.monitoring.yml up -d"

  # Start monitoring on remote2 if applicable
  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "📊 Starting monitoring on remote2..."
    run_remote "$REMOTE2_HOST" "$REMOTE2_USER" \
      "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME \
       -f $REMOTE_DIR/docker-compose.monitoring.yml up -d"
  fi

  # Start sidecars on remote1
  echo "🛡️  Starting sidecars on remote1..."
  if ! remote_compose_up "$REMOTE1_HOST" "$REMOTE1_USER" \
      "$REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote1.yml"; then
    echo "❌ Failed to start sidecars on remote1."
    exit 1
  fi

  # Start sidecars on remote2 if applicable
  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "🛡️  Starting sidecars on remote2..."
    if ! remote_compose_up "$REMOTE2_HOST" "$REMOTE2_USER" \
        "$REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote2.yml"; then
      echo "❌ Failed to start sidecars on remote2."
      exit 1
    fi
  fi

else
# ── LOCAL MODE: everything runs on d-mutra ────────────────────────────────────

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" up -d

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d

  if wait_for_secaas; then
    echo "🔄 Synchronizing agent signatures to PostgreSQL..."
    curl -X POST "$SECAAS_URL/sync-agents?n_robots=$N_ROBOTS_VAL" -H "Content-Type: application/json" | jq
    echo "✅ Sync complete."
  else
    exit 1
  fi

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" up -d

fi
# ══════════════════════════════════════════════════════════════════════════════

echo "🎉 All done."
