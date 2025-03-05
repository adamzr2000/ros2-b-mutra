# Blockchain-based Mutual Remote Attestation for ROS2 Executables

**Author:** Adam Zahir Rodriguez

## Installation
Before getting started, make sure you have the [Docker](https://docs.docker.com/engine/install/ubuntu/) and [Docker Compose](https://docs.docker.com/compose/install/linux/) installed on your system.

1. Clone the repository:
```bash
git clone git@github.com:adamzr2000/turtlebot3-service-remote-attestaition.git
```

2. Build Docker Images:
Go to the [dockerfiles](./dockerfiles) directory and run the `./build.sh` scripts for each image:

- `dlt-node`: Based on [Go-Ethereum (Geth)](https://geth.ethereum.org/docs) software, serving as nodes within the peer-to-peer blockchain network. (detailed info [here](./dockerfiles/dlt-node/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `eth-netstats`: Dashboard for monitoring Geth nodes. (detailed info [here](./dockerfiles/eth-netstats/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `truffle`: Development framework for Ethereum-based blockchain applications. It provides a suite of tools that allows developers to write, test, and deploy smart contracts. (detailed info [here](./dockerfiles/eth-netstats/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `gazebo`: Robot simulation tool (detailed info [here](./dockerfiles/gazebo-vnc/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `turtlebot3`: Robot simulation model in Gazebo (detailed info [here](./dockerfiles/turtlebot3/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

## Blockchain Network Setup
1. Initialize the network
```bash
cd dlt-network/geth-poa
sudo ./start_geth_poa_network.sh
```

Access the `eth-netsats` web interface for additional information at [http://10.5.99.99:3000](http://localhost:3000)

2. Deploy the [MasMutualAttestation](./smart-contracts/contracts/MasMutualAttestation.sol) smart contract
```bash
./deploy_smart_contract.sh --node-ip 10.0.1.1 --ws-port 3334 
```

3. Stop the network
```bash
cd dlt-network/geth-poa
sudo ./stop_geth_poa_network.sh
```

## Usage

1. Start the experimental scenario
```bash
docker compose up -d
```

2. Run blockchain-based attestation on SECaaS
Execute the mutual attestation script inside the `secaas` container:

```bash
docker exec -it secaas sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant secaas"
```

3. Run blockchain-based attestation on Robots
Execute the mutual attestation script for each `robot` container:

```bash
docker exec -it robot1 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
```

```bash
docker exec -it robot2 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
```

```bash
docker exec -it robot3 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
```

4. Stop the experimental scenario
```bash
docker compose down
```

Run the `./cleanup.sh` script to clean up all resources after testing.

## Utilities

### Create scenario configuration
```bash
python3 create_experimental_scenario.py
```

> Note: This script generates agent and SECaaS configuration files, assigns blockchain credentials, and creates a `docker-compose.yml` file for deployment.

### Delete scenario configuration
```bash
./cleanup.sh
```

### Reset attestation chain in smart contract
```bash
docker exec -it secaas sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant secaas --reset-chain"
```

### Retrieve blockchain transaction receipt
```bash
docker exec -it secaas sh -c "cd /home/agent/scripts && python3 get_tx_receipt.py --eth-node-url ws://10.0.1.1:3334 --tx-hash <hash>"
```

### Remove registered agent from the smart contract
```bash
docker exec -it robot1 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent --remove-agent"
```

### Simulate failed attestation
```bash
docker exec -it robot1 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap --fail-attestation"
```

