#!/bin/bash
set -e

# Define password
PASSWORD="netcom;"

# Use sudo with password piped into stdin
echo "$PASSWORD" | sudo -S docker compose down

cd blockchain/quorum-test-network
./remove.sh