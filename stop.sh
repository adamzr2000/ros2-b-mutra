#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# One Compose project name for ALL stacks
PROJECT_NAME="ros2-b-mutra"

PASSWORD="netcom;"

# Multi-host constants
LOCAL_LIMIT=8
REMOTE_HOST="10.5.1.21"
REMOTE_USER="desire6g"
REMOTE_PASSWORD="desire6g2024;"
REMOTE_DIR="~/ros2-b-mutra"

# Read N_ROBOTS from .env to decide single-host vs multi-host teardown
ENV_FILE="$SCRIPT_DIR/.env"
N_ROBOTS_VAL=4
if [[ -f "$ENV_FILE" ]]; then
  _n=$(grep -oP '(?<=^N_ROBOTS=)\d+' "$ENV_FILE" || true)
  [[ -n "$_n" ]] && N_ROBOTS_VAL="$_n"
fi

# Stop monitoring (d-mutra always; remote host in multi-host mode)
echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" down

if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  ssh "$REMOTE_USER@$REMOTE_HOST" \
    "cd $REMOTE_DIR && echo '$REMOTE_PASSWORD' | sudo -S docker compose -p $PROJECT_NAME \
     -f $REMOTE_DIR/docker-compose.monitoring.yml down" || true
fi

if (( N_ROBOTS_VAL > LOCAL_LIMIT )); then
  echo "🌐 Multi-host teardown (N=$N_ROBOTS_VAL)..."

  # Stop remote sidecars and robots on remote host
  ssh "$REMOTE_USER@$REMOTE_HOST" \
    "cd $REMOTE_DIR && \
     echo '$REMOTE_PASSWORD' | sudo -S docker compose -p $PROJECT_NAME --project-directory $REMOTE_DIR \
       -f $REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote.yml down && \
     echo '$REMOTE_PASSWORD' | sudo -S docker compose -p $PROJECT_NAME --project-directory $REMOTE_DIR \
       -f $REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote.yml down"

  # Stop local sidecars and robots on d-mutra
  echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" --project-directory "$SCRIPT_DIR" \
    -f "$SCRIPT_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-local.yml" down
  echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" --project-directory "$SCRIPT_DIR" \
    -f "$SCRIPT_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-local.yml" down
else
  echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" down
  echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" down
fi

echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" down

# Remove blockchain network
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./remove.sh


# Reset EventWatcher checkpoints (seen_keys/from_block)
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting EventWatcher checkpoints..."
  ( cd "$CHECKPOINT_DIR" && echo "$PASSWORD" | sudo -S bash "./$(basename "$RESET_SCRIPT")" )
else
  echo "⚠️  Checkpoints reset skipped (missing $RESET_SCRIPT or $CHECKPOINT_DIR)"
fi
