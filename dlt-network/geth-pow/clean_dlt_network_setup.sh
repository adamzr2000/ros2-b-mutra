#!/bin/bash

# Define configuration directory and log directory variables for better scalability
CONFIG_DIR="config"
LOG_DIR="logs"
ENV_FILE=".env"
DOCKER_COMPOSE_FILE="docker-compose.yml"

# Remove all directories named "nodeX"
for dir in "$CONFIG_DIR"/node*; do
  if [ -d "$dir" ]; then
    echo "Removing directory: $dir"
    rm -rf "$dir"
  fi
done

# Remove all nodeX_start.sh files
for file in "$CONFIG_DIR"/node*_start.sh; do
  if [ -f "$file" ]; then
    echo "Removing file: $file"
    rm -f "$file"
  fi
done

# Remove the "bootnode" directory if it exists
if [ -d "$CONFIG_DIR/bootnode" ]; then
  echo "Removing directory: $CONFIG_DIR/bootnode"
  rm -rf "$CONFIG_DIR/bootnode"
fi

# Remove the "logs" directory if it exists
if [ -d "$LOG_DIR" ]; then
  echo "Removing directory: $LOG_DIR"
  rm -rf "$LOG_DIR"
fi

# Remove "bootnode_start.sh" file
BOOTNODE_START_FILE="$CONFIG_DIR/bootnode_start.sh"
if [ -f "$BOOTNODE_START_FILE" ]; then
  echo "Removing file: $BOOTNODE_START_FILE"
  rm -f "$BOOTNODE_START_FILE"
fi

# Remove "genesis.json" file
GENESIS_FILE="$CONFIG_DIR/genesis.json"
if [ -f "$GENESIS_FILE" ]; then
  echo "Removing file: $GENESIS_FILE"
  rm -f "$GENESIS_FILE"
fi

# Remove the ".env" file
if [ -f "$ENV_FILE" ]; then
  echo "Removing file: $ENV_FILE"
  rm -f "$ENV_FILE"
fi

# Remove the "docker-compose" file
if [ -f "$DOCKER_COMPOSE_FILE" ]; then
  echo "Removing file: $DOCKER_COMPOSE_FILE"
  rm -f "$DOCKER_COMPOSE_FILE"
fi

echo "Cleanup complete."
