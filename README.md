# Securing the MAS with DLT: Integrity and Inter-agent communications

## Overview
This repository contains scripts and Dockerfiles to secure Multi-Agent Systems (MAS) using Distributed Ledger Technologies (DLTs), specifically the combination of Ethereum blockchain and smart contracts.

**Author:** Adam Zahir Rodriguez

## Installation
Before getting started, make sure you have the [Docker](https://docs.docker.com/engine/install/ubuntu/) and [Docker Compose](https://docs.docker.com/compose/install/linux/) installed on your system.

1. Clone the repository:
```bash
git clone git@github.com:adamzr2000/turtlebot3-service-remote-attestaition.git
```

2. Build Docker Images:
Go to the [docker-images](./dockerfiles) directory and run the `./build.sh` scripts for each image:

- `dlt-node`: Based on [Go-Ethereum (Geth)](https://geth.ethereum.org/docs) software, serving as nodes within the peer-to-peer blockchain network. (detailed info [here](./dockerfiles/dlt-node/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `eth-netstats`: Dashboard for monitoring Geth nodes. (detailed info [here](./dockerfiles/eth-netstats/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `truffle`: Development framework for Ethereum-based blockchain applications. It provides a suite of tools that allows developers to write, test, and deploy smart contracts. (detailed info [here](./dockerfiles/eth-netstats/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `turtlebot3`: Mobile robot used in the simulation (detailed info [here](./dockerfiles/turtlebot3/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available

- `gazebo-vnc`: Robot simulator (detailed info [here](./dockerfiles/gazebo-vnc/)). ![#00FF00](https://via.placeholder.com/15/00ff00/000000?text=+) Available


# Blockchain Network Setup
Create a blockchain network using `dlt-node` containers.

1. Initialize the network
```bash
cd dlt-network/geth-poa
sudo ./start_geth_poa_network.sh
```

Access the `eth-netsats` web interface for additional information at [http://10.5.99.99:3000](http://localhost:3000)

2. Stop the network
```bash
sudo ./stop_geth_poa_network.sh
```

# Usage

1. Deploy the MasMutualAttestation SC to the blockchain Network:

```bash
./deploy_smart_contract.sh --node-ip 10.0.1.1 --ws-port 3334 
```

2. Create scenario configuration

```bash
python3 create_experimental_scenario.py
```

> Note: This generates JSON configuration files containing agent settings. Additionally, a Docker Compose file is created, which will be used to launch the experimental scenario.

4. Start the experimental scenario
```bash
docker compose up -d
```

# Mutual Attestation

4. SECaaS
In `secaas` container, run:

```bash
docker exec -it secaas sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant secaas"
```

4. Robots
In `robot` containers, run:

```bash
docker exec -it robot1 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent"
docker exec -it robot2 sh -c "cd /home/agent/scripts && python3 mas_mutual_attestation.py --participant agent"

...
```

4. Stop the experimental scenario
```bash
docker compose down
```

## Cleanup
Run the following script in the app directory to clean up all resources after testing.

```bash
./cleanup.sh
```

