#!/bin/bash
set -e

# Define password
PASSWORD="netcom;"

# Use sudo with password piped into stdin
echo "$PASSWORD" | sudo -S docker compose down

cd blockchain-network/geth-poa
echo "$PASSWORD" | sudo -S ./stop_geth_net.sh

docker container rm sidecar-measurer-robot3 sidecar-measurer-robot2 robot2 robot3 sidecar-controller-robot2 sidecar-controller-robot3
