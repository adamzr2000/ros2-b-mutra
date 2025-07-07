#!/bin/bash
set -e

# Define password
PASSWORD="netcom;"

# Use sudo with password piped into stdin
echo "$PASSWORD" | sudo -S docker compose down

cd dlt-network/geth-poa
echo "$PASSWORD" | sudo -S ./stop_geth_poa_network.sh
