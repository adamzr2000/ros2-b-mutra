#!/bin/bash

# Execute the geth init command to initialize the data directory with genesis.json
echo "== Initializing Geth with genesis.json =="
output=$(geth init --datadir node1 genesis.json)
echo "== Geth initialization output =="
echo "$output"

# Read environment variables from node specific .env file
source .env

echo "== Environment variables loaded: =="
env | grep -E "IP_NODE_1|WS_PORT_NODE_1|ETH_PORT_NODE_1|BOOTNODE_URL|NETWORK_ID|RPC_PORT_NODE_1|ETHERBASE_NODE_1|WS_SECRET|ETH_NETSATS_IP|ETH_NETSATS_PORT"

# Define the command
echo "== Defining Geth command =="
# command="geth --identity 'node1' --syncmode 'full' --ws --ws.addr $IP_NODE_1 --ws.port $WS_PORT_NODE_1 --datadir node1 --port $ETH_PORT_NODE_1 --bootnodes $BOOTNODE_URL --ws.api 'eth,net,web3,personal,miner,admin' --networkid $NETWORK_ID --nat 'any' --allow-insecure-unlock --authrpc.port $RPC_PORT_NODE_1 --ipcdisable --unlock $ETHERBASE_NODE_1 --password password.txt --mine --snapshot=false --miner.etherbase $ETHERBASE_NODE_1 --ethstats node1:$WS_SECRET@$ETH_NETSATS_IP:$ETH_NETSATS_PORT --miner.threads 1"  
command="geth --identity 'node1' --syncmode 'full' --ws --ws.addr $IP_NODE_1 --ws.port $WS_PORT_NODE_1 --datadir node1 --port $ETH_PORT_NODE_1 --bootnodes $BOOTNODE_URL --ws.api 'eth,net,web3,personal,miner,admin' --networkid $NETWORK_ID --nat 'any' --allow-insecure-unlock --authrpc.port $RPC_PORT_NODE_1 --ipcdisable --unlock $ETHERBASE_NODE_1 --password password.txt --snapshot=false --miner.etherbase $ETHERBASE_NODE_1 --ethstats node1:$WS_SECRET@$ETH_NETSATS_IP:$ETH_NETSATS_PORT --preload /dlt-network/startmine.js"

# Add verbosity option to the command if logs need to be saved
if [ "$SAVE_LOGS" == "y" ] || [ "$SAVE_LOGS" == "Y" ]; then
  command="$command --verbosity 3 >> ./logs/node1.log 2>&1"
else
  command="$command"
fi

echo "== Final Geth command to be executed =="
echo "$command"

# Execute the command
echo "== Executing Geth command =="
eval $command

# Execute the command
eval $command
