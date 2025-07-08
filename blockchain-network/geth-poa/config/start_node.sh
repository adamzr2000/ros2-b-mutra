#!/usr/bin/env bash
set -euo pipefail

#------------------------------------------------------------------------
# MODE SWITCH:
#   To start a bootnode, set:
#     export IDENTITY=bootnode
#     export BOOTNODE_IP=<ip>
#     export BOOTNODE_PORT=<port>
#
#   To start a Geth node, set:
#     export IDENTITY=nodeX             # node1, node2, etc.
#     export IP_ADDR=<ip>              # bind IP for RPC
#     export WS_PORT=<port>            # used for both WS and HTTP RPC
#     export ETH_PORT=<port>           # P2P port
#     export RPC_PORT=<port>           # internal use for authrpc
#     export ETHERBASE=<0x...>         # miner account
#     export BOOTNODE_URL=<enode://...>
#     export NETWORK_ID=<int>
#     export WS_SECRET=<string>
#     export ETH_NETSATS_IP=<ip>
#     export ETH_NETSATS_PORT=<port>
#     [optional] RPC_PROTOCOL=ws|http  # default is ws
#     [optional] SAVE_LOGS=Y           # log to logs/nodeX.log
#------------------------------------------------------------------------

: "${IDENTITY:?set IDENTITY (nodeX or bootnode)}"

if [[ "${IDENTITY,,}" == "bootnode" ]]; then
  : "${BOOTNODE_IP:?}" "${BOOTNODE_PORT:?}"
  echo "🚀 Starting bootnode on ${BOOTNODE_IP}:${BOOTNODE_PORT}"
  exec bootnode -nodekey ./bootnode/boot.key -verbosity 9 -addr "${BOOTNODE_IP}:${BOOTNODE_PORT}"
fi

: "${IP_ADDR:?}" "${WS_PORT:?}" "${ETH_PORT:?}" "${RPC_PORT:?}" "${ETHERBASE:?}"
: "${BOOTNODE_URL:?}" "${NETWORK_ID:?}" "${WS_SECRET:?}" "${ETH_NETSATS_IP:?}" "${ETH_NETSATS_PORT:?}"

RPC_PROTOCOL="${RPC_PROTOCOL:-ws}"
DATADIR="$IDENTITY"
GENESIS_FILE="${GENESIS_FILE:-genesis.json}"

echo "🔧 Initializing $DATADIR with $GENESIS_FILE"
geth init --datadir "$DATADIR" "$GENESIS_FILE"

cmd=(
  geth
  --identity "$IDENTITY"
  --syncmode full
  --datadir "$DATADIR"
  --port "$ETH_PORT"
  --bootnodes "$BOOTNODE_URL"
  --networkid "$NETWORK_ID"
  --nat any
  --allow-insecure-unlock
  --authrpc.port "$RPC_PORT"
  --ipcdisable
  --unlock "$ETHERBASE"
  --password password.txt
  --mine
  --snapshot=false
  --miner.etherbase "$ETHERBASE"
  --ethstats "$IDENTITY:$WS_SECRET@$ETH_NETSATS_IP:$ETH_NETSATS_PORT"
)

if [[ "$RPC_PROTOCOL" == "http" ]]; then
  echo "🌐 Using HTTP on ${IP_ADDR}:${WS_PORT}"
  cmd+=(--http --http.addr "$IP_ADDR" --http.port "$WS_PORT" --http.api "eth,net,web3,personal,miner,admin,clique" --http.corsdomain "*")
else
  echo "🌐 Using WebSocket on ${IP_ADDR}:${WS_PORT}"
  cmd+=(--ws --ws.addr "$IP_ADDR" --ws.port "$WS_PORT" --ws.api "eth,net,web3,personal,miner,admin,clique")
fi

echo "🚀 Starting node $IDENTITY (P2P: $ETH_PORT, RPC: $RPC_PROTOCOL://$IP_ADDR:$WS_PORT)"

if [[ "${SAVE_LOGS:-n}" =~ ^[Yy]$ ]]; then
  mkdir -p logs
  cmd+=(--verbosity 3)
  echo "📝 Logging to logs/${IDENTITY}.log"
  "${cmd[@]}" >> "logs/${IDENTITY}.log" 2>&1
else
  "${cmd[@]}"
fi
