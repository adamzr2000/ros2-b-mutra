#!/usr/bin/env bash
set -euo pipefail

#------------------------------------------------------------------------
# MODE SWITCH:
#   If you want to start a bootnode, do:
#     export IDENTITY=bootnode
#     export BOOTNODE_IP=<addr>
#     export BOOTNODE_PORT=<port>
#   Otherwise (regular geth node):
#     export IDENTITY=node1      # or node2, etc.
#     export IP_ADDR=<addr>
#     export WS_PORT=<ws port>
#     export ETH_PORT=<p2p port>
#     export RPC_PORT=<rpc port>
#     export ETHERBASE=<miner addr>
#     export BOOTNODE_URL=<enode://…>
#     export NETWORK_ID=<chain id>
#     export WS_SECRET=<stats secret>
#     export ETH_NETSATS_IP=<stats host>
#     export ETH_NETSATS_PORT=<stats port>
#
# Optional for both:
#     export GENESIS_FILE=<file>    # defaults to "genesis.json"
#     export SAVE_LOGS=Y            # turn on verbosity+logs
#------------------------------------------------------------------------

: "${IDENTITY:?set IDENTITY (node1, node2 or bootnode)}"

if [[ "${IDENTITY,,}" == "bootnode" ]]; then
  #––– BOOTNODE MODE –––
  : "${BOOTNODE_IP:?set BOOTNODE_IP}"
  : "${BOOTNODE_PORT:?set BOOTNODE_PORT}"
  echo "🚀 Starting bootnode on ${BOOTNODE_IP}:${BOOTNODE_PORT}"
  exec bootnode \
    -nodekey ./bootnode/boot.key \
    -verbosity 9 \
    -addr "${BOOTNODE_IP}:${BOOTNODE_PORT}"
fi

#––– GETH NODE MODE –––
: "${IP_ADDR:?set IP_ADDR}"
: "${WS_PORT:?set WS_PORT}"
: "${ETH_PORT:?set ETH_PORT}"
: "${RPC_PORT:?set RPC_PORT}"
: "${ETHERBASE:?set ETHERBASE}"
: "${BOOTNODE_URL:?set BOOTNODE_URL}"
: "${NETWORK_ID:?set NETWORK_ID}"
: "${WS_SECRET:?set WS_SECRET}"
: "${ETH_NETSATS_IP:?set ETH_NETSATS_IP}"
: "${ETH_NETSATS_PORT:?set ETH_NETSATS_PORT}"

GENESIS_FILE="${GENESIS_FILE:-genesis.json}"
DATADIR="$IDENTITY"

echo "🔧 Initializing $DATADIR with $GENESIS_FILE"
geth init --datadir "$DATADIR" "$GENESIS_FILE"

cmd=(
  geth
    --identity       "$IDENTITY"
    --syncmode       full
    --ws
    --ws.addr        "$IP_ADDR"
    --ws.port        "$WS_PORT"
    --datadir        "$DATADIR"
    --port           "$ETH_PORT"
    --bootnodes      "$BOOTNODE_URL"
    --ws.api         "eth,net,web3,personal,miner,admin,clique"
    --networkid      "$NETWORK_ID"
    --nat            any
    --allow-insecure-unlock
    --authrpc.port   "$RPC_PORT"
    --ipcdisable
    --unlock         "$ETHERBASE"
    --password       password.txt
    --mine
    --snapshot=false
    --miner.etherbase "$ETHERBASE"
    --ethstats       "$IDENTITY:$WS_SECRET@$ETH_NETSATS_IP:$ETH_NETSATS_PORT"
)

if [[ "${SAVE_LOGS:-n}" =~ ^[Yy]$ ]]; then
  mkdir -p logs
  cmd+=(--verbosity 3)
  echo "📝 Logging to logs/${IDENTITY}.log"
  "${cmd[@]}" >> "logs/${IDENTITY}.log" 2>&1
else
  "${cmd[@]}"
fi
