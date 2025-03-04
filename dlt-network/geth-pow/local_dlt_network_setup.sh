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
    read -r $variable_name  # Remove the double quotes
    if [[ ! ${!variable_name} =~ $validation_pattern ]]; then  # Use ${!variable_name} to access the variable's value
      echo "Invalid input. Please try again."
    else
      break
    fi
  done
}

# Prompt for the number of geth nodes (>0)
prompt_and_validate_input "Please enter the number of geth nodes for the network [>0]:" numNodes '^[1-9][0-9]*$'

# Prompt for chainID value (>0)
prompt_and_validate_input "Please enter the 'chainID' value for genesis.json [>0]:" chainID '^[1-9][0-9]*$'

# Prompt for log saving option (y/n)
prompt_and_validate_input "Do you want to save logs in a .log file? (y/n):" saveLogs '^[ynYN]$'

echo "Number of nodes: $numNodes"
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
DLT_SUBNET=10.0.0.0/16
WS_SECRET=mysecret
ETH_NETSATS_IP=10.0.0.2
ETH_NETSATS_PORT=3000
BOOTNODE_IP=10.0.0.3
BOOTNODE_PORT=30301
SAVE_LOGS=$saveLogs
EOF

# Generate node addresses and update the .env file
declare -a addresses
alloc=""

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

  # Append address and alloc sections
  alloc+='"'$addr'": { "balance": "100000000000000000000" },'
  
  # Prepare node start script
  cat << EOF > "$CONFIG_DIR/node${i}_start.sh"
#!/bin/bash

# Execute the geth init command to initialize the data directory with genesis.json
output=\$(geth init --datadir node$i genesis.json)
echo "\$output"

# Read environment variables from node specific .env file
source .env

# Define the command
command="geth --identity 'node$i' --syncmode 'full' --nodiscover --ws --ws.addr \$IP_NODE_$i --ws.port \$WS_PORT_NODE_$i --datadir node$i --port \$ETH_PORT_NODE_$i --bootnodes \$BOOTNODE_URL --ws.api 'eth,net,web3,personal,miner,admin' --networkid \$NETWORK_ID --nat 'any' --allow-insecure-unlock --authrpc.port \$RPC_PORT_NODE_$i --ipcdisable --unlock \$ETHERBASE_NODE_$i --password password.txt --mine --snapshot=false --miner.etherbase \$ETHERBASE_NODE_$i --ethstats node$i:\$WS_SECRET@\$ETH_NETSATS_IP:\$ETH_NETSATS_PORT --miner.threads 1"  

# Add verbosity option to the command if logs need to be saved
if [ "\$SAVE_LOGS" == "y" ] || [ "\$SAVE_LOGS" == "Y" ]; then
  command="\$command --verbosity 3 >> ./logs/node$i.log 2>&1"
else
  command="\$command"
fi

# Execute the command
eval \$command
EOF

  chmod +x "$CONFIG_DIR/node${i}_start.sh"

  echo "node$i created and configured."
done

# Remove trailing comma from alloc
alloc=${alloc::-1}

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
    "berlinBlock": 0,
    "londonBlock": 0,
    "ethash": {}
  },
  "difficulty": "0x01",
  "mixhash": "0x0000000000000000000000000000000000000000000000000000000000000000",
  "gasLimit": "8000000",
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

# Prepare bootnode start script
cat << EOF > "$CONFIG_DIR/bootnode_start.sh"
#!/bin/bash

# Read environment variables from bootnode specific .env file
source .env

# Start the bootnode service.
bootnode -nodekey ./bootnode/boot.key -verbosity 9 -addr \$BOOTNODE_IP:\$BOOTNODE_PORT
EOF

chmod +x "$CONFIG_DIR/bootnode_start.sh"

python3 "$CONFIG_DIR/private_key_decrypt.py"


# Generate docker-compose.yml file
touch $DOCKER_COMPOSE_FILE

# Write the base structure of docker-compose file
cat << EOF > $DOCKER_COMPOSE_FILE
version: '3'
services:
  bootnode:
    image: dlt-node:geth-pow
    container_name: bootnode
    hostname: bootnode
    env_file: .env
    command: ./bootnode_start.sh
    volumes:
      - "./$CONFIG_DIR:/dlt-network/"
    networks:
      dlt_network:
        ipv4_address: \${BOOTNODE_IP}
    restart: always
EOF

# Add each node to the docker-compose file
for (( i=1; i<=$numNodes; i++ )); do
  cat << EOF >> $DOCKER_COMPOSE_FILE

  node$i:
    image: dlt-node:geth-pow
    container_name: node$i
    hostname: node$i
    depends_on:
      - bootnode
    env_file: .env
    command: ./node${i}_start.sh
    volumes:
      - "./$CONFIG_DIR:/dlt-network/"
    networks:
      dlt_network:
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
      dlt_network:
        ipv4_address: \${ETH_NETSATS_IP}
    restart: always

networks:
  dlt_network:
    name: dlt_network
    ipam:
      driver: default
      config:
        - subnet: \${DLT_SUBNET}
EOF

echo "Setup completed."
