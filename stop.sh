#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# One Compose project name for ALL stacks
PROJECT_NAME="ros2-b-mutra"

# Stop experimental scenario (all compose stacks)
PASSWORD="netcom;"
echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" down
echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" down
echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" down
echo "$PASSWORD" | sudo -S docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" down

# Remove blockchain network
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
./remove.sh

# Reset EventWatcher checkpoints (seen_keys/from_block)
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting EventWatcher checkpoints..."
  ( cd "$CHECKPOINT_DIR" && bash "./$(basename "$RESET_SCRIPT")" )
else
  echo "⚠️  Checkpoints reset skipped (missing $RESET_SCRIPT or $CHECKPOINT_DIR)"
fi
