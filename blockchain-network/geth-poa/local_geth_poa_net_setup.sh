#!/bin/bash

# Define variables for better scalability
CONFIG_DIR="config"
LOG_DIR="logs"
ENV_FILE=".env"
DOCKER_COMPOSE_FILE="docker-compose.yml"

# Function to prompt the user for input and validate it
prompt_and_validate_input() {
  local prompt_message="$1"
  local variable_name="$2"
  local validation_pattern="$3"

  while true; do
    echo "$prompt_message"
    read -r $variable_name
    if [[ ! ${!variable_name} =~ $validation_pattern ]]; then
      echo "Invalid input. Please try again."
    else
      break
    fi
  done
}

# Prompt for the number of geth nodes (>0)
prompt_and_validate_input "Please enter the number of geth nodes for the network [>0]:" numNodes '^[1-9][0-9]*$'
# Prompt for period value (>=0)
prompt_and_validate_input "Please enter the 'period' value (average time(s) interval for adding new blocks to the blockchain) [>=0]:" period '^[0-9]+$|^0$'
# Prompt for chainID value (>0)
prompt_and_validate_input "Please enter the 'chainID' value for genesis.json [>0]:" chainID '^[1-9][0-9]*$'
# Prompt for log saving option (y/n)
prompt_and_validate_input "Do you want to save logs in a .log file? (y/n):" saveLogs '^[ynYN]$'

echo "Number of nodes: $numNodes"
echo "Block period: $period seconds"
echo "Chain ID: $chainID"
echo "Save logs: $saveLogs"

# Create the logs directory
mkdir -p $LOG_DIR

# Initialize the .env file
touch $ENV_FILE

# Write global environment variables to the .env file
cat << EOF > $ENV_FILE
# Global configuration
NETWORK_ID=$chainID
BLOCKCHAIN_SUBNET=10.0.0.0/16
WS_SECRET=mysecret
ETH_NETSATS_IP=10.0.0.2
ETH_NETSATS_PORT=3000
BOOTNODE_IP=10.0.0.3
BOOTNODE_PORT=30301
SAVE_LOGS=$saveLogs
RPC_PROTOCOL=ws
EOF

# Generate node addresses and update the .env file
declare -a addresses
alloc=""
extraData="0x0000000000000000000000000000000000000000000000000000000000000000"

for (( i=1; i<=$numNodes; i++ )); do
  mkdir -p "$CONFIG_DIR/node$i"
  
  # Generate a new account
  addr=$(geth --datadir "$CONFIG_DIR/node$i" account new --password "$CONFIG_DIR/password.txt" 2>&1 | grep "Public address of the key" | awk '{print $NF}')
  addresses+=("$addr")
  
  # Append node configuration to the .env file
  cat << EOF >> $ENV_FILE
# Node $i configuration
ETHERBASE_NODE_$i=$addr
IP_NODE_$i=10.0.1.$i
WS_PORT_NODE_$i=$((3333 + $i))
RPC_PORT_NODE_$i=$((8550 + $i))
ETH_PORT_NODE_$i=$((30302 + $i))
WS_NODE_${i}_URL=ws://\${IP_NODE_$i}:\${WS_PORT_NODE_$i}
EOF

  # Append address to extraData and alloc sections
  extraData+="${addr#'0x'}"
  alloc+='"'$addr'": { "balance": "100000000000000000000" },'

  echo "node$i created and configured."
done

# Remove trailing comma from alloc
alloc=${alloc::-1}

# Add 65 zero bytes at the end of extraData
extraData+="0000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000000"

# Create genesis.json file
cat << EOF > "$CONFIG_DIR/genesis.json"
{
  "config": {
    "chainId": $chainID,
    "homesteadBlock": 0,
    "eip150Block": 0,
    "eip155Block": 0,
    "eip158Block": 0,
    "byzantiumBlock": 0,
    "constantinopleBlock": 0,
    "petersburgBlock": 0,
    "istanbulBlock": 0,
    "muirGlacierBlock": 0,
    "berlinBlock": 0,
    "londonBlock": 0,
    "arrowGlacierBlock": 0,
    "grayGlacierBlock": 0,
    "clique": {
      "period": $period,
      "epoch": 30000
    }
  },
  "difficulty": "1",
  "gasLimit": "8000000",
  "extraData": "$extraData",
  "alloc": {
    $alloc
  }
}
EOF

# Create bootnode
mkdir -p "$CONFIG_DIR/bootnode" && bootnode -genkey "$CONFIG_DIR/bootnode/boot.key"
bootnode_key=$(bootnode -writeaddress -nodekey "$CONFIG_DIR/bootnode/boot.key")

# Append bootnode URL to .env file
cat << EOF >> $ENV_FILE
# Bootnode configuration
BOOTNODE_KEY=$bootnode_key
BOOTNODE_URL=enode://\$BOOTNODE_KEY@\$BOOTNODE_IP:\$BOOTNODE_PORT

# Private keys
EOF

python3 "$CONFIG_DIR/private_key_decrypt.py"

# Generate docker-compose.yml file
touch $DOCKER_COMPOSE_FILE

# Write the base structure of docker-compose file
cat << EOF > $DOCKER_COMPOSE_FILE
# version: '3'
x-common-commands:
  node_entrypoint: &node_entrypoint >
    bash -c "
    ./start_node.sh
    "
services:
  bootnode:
    image: geth-node:poa
    container_name: bootnode
    hostname: bootnode
    environment:
      - IDENTITY=bootnode
      - BOOTNODE_IP=\${BOOTNODE_IP}
      - BOOTNODE_PORT=\${BOOTNODE_PORT}
    command: *node_entrypoint
    volumes:
      - "./$CONFIG_DIR:/src/"
    networks:
      blockchain_network:
        ipv4_address: \${BOOTNODE_IP}
    restart: always
EOF

# Add each node to the docker-compose file
for (( i=1; i<=$numNodes; i++ )); do
  cat << EOF >> $DOCKER_COMPOSE_FILE

  node$i:
    image: geth-node:poa
    container_name: node$i
    hostname: node$i
    depends_on:
      - bootnode
    environment:
      - IDENTITY=node$i
      - ETHERBASE=\${ETHERBASE_NODE_$i}
      - IP_ADDR=\${IP_NODE_$i}
      - WS_PORT=\${WS_PORT_NODE_$i}
      - RPC_PORT=\${RPC_PORT_NODE_$i}
      - ETH_PORT=\${ETH_PORT_NODE_$i}
      - BOOTNODE_URL=\${BOOTNODE_URL}
      - NETWORK_ID=\${NETWORK_ID}
      - WS_SECRET=\${WS_SECRET}
      - ETH_NETSATS_IP=\${ETH_NETSATS_IP}
      - ETH_NETSATS_PORT=\${ETH_NETSATS_PORT}
      - RPC_PROTOCOL=\${RPC_PROTOCOL}
    command: *node_entrypoint
    volumes:
      - "./$CONFIG_DIR:/src/"
    networks:
      blockchain_network:
        ipv4_address: \${IP_NODE_$i}
    restart: always
EOF
done

# Add eth-netstats service
cat << EOF >> $DOCKER_COMPOSE_FILE

  eth-netstats:
    image: eth-netstats
    container_name: eth-netstats
    depends_on:
      - node1
    ports:
      - "\${ETH_NETSATS_PORT}:\${ETH_NETSATS_PORT}"
    networks:
      blockchain_network:
        ipv4_address: \${ETH_NETSATS_IP}
    restart: always

networks:
  blockchain_network:
    name: blockchain_network
    ipam:
      driver: default
      config:
        - subnet: \${BLOCKCHAIN_SUBNET}
EOF

echo "Setup completed."
