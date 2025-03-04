#!/bin/bash

# Default consensus algorithm
CONSENSUS="geth-poa"

# Parse the --consensus flag
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --consensus) CONSENSUS="$2"; shift ;; # Capture the consensus type
        *) echo "Unknown parameter passed: $1"; exit 1 ;;
    esac
    shift
done

# Check if the consensus type is valid
if [[ "$CONSENSUS" != "geth-poa" && "$CONSENSUS" != "geth-pow" ]]; then
    echo "Invalid consensus type. Please specify either 'geth-poa' or 'geth-pow'."
    exit 1
fi

# Assemble the docker image based on the consensus type.
echo "Building dlt-node docker image with consensus: $CONSENSUS."
docker build -t dlt-node:$CONSENSUS -f Dockerfile-$CONSENSUS .
