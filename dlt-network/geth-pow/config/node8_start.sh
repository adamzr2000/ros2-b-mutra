#!/bin/bash

# Execute the geth init command to initialize the data directory with genesis.json
output=$(geth init --datadir node8 genesis.json)
echo "$output"

# Read environment variables from node specific .env file
source .env

# Define the command
command="geth --identity 'node8' --syncmode 'full' --nodiscover --ws --ws.addr $IP_NODE_8 --ws.port $WS_PORT_NODE_8 --datadir node8 --port $ETH_PORT_NODE_8 --bootnodes $BOOTNODE_URL --ws.api 'eth,net,web3,personal,miner,admin' --networkid $NETWORK_ID --nat 'any' --allow-insecure-unlock --authrpc.port $RPC_PORT_NODE_8 --ipcdisable --unlock $ETHERBASE_NODE_8 --password password.txt --mine --snapshot=false --miner.etherbase $ETHERBASE_NODE_8 --ethstats node8:$WS_SECRET@$ETH_NETSATS_IP:$ETH_NETSATS_PORT --miner.threads 1"  

# Add verbosity option to the command if logs need to be saved
if [ "$SAVE_LOGS" == "y" ] || [ "$SAVE_LOGS" == "Y" ]; then
  command="$command --verbosity 3 >> ./logs/node8.log 2>&1"
else
  command="$command"
fi

# Execute the command
eval $command
