# Blockchain-based Mutual Remote Attestation (B-MUTRA) for ROS2 Executables

**Author(s):** Adam Zahir Rodriguez and Mark Angoustures

## Installation
Before getting started, make sure you have the [Docker](https://docs.docker.com/engine/install/ubuntu/) and [Docker Compose](https://docs.docker.com/compose/install/linux/) installed on your system.

1. Clone the repository:
```bash
git clone git@github.com:adamzr2000/ros2-b-mutra.git
```

2. Build Docker Images:
```bash
cd dockerfiles
make
```

The following modules will be built and tagged locally:

| Module                 | Description                                                                                                     | Status       |
|------------------------|-----------------------------------------------------------------------------------------------------------------|--------------|
| **hardhat**            | Development framework for Ethereum-based blockchain applications ([details](./dockerfiles/hardhat))         | ✅ Available |
| **gazebo-vnc**         | Robot simulation tool ([details](./dockerfiles/gazebo-vnc))       | ✅ Available |
| **turtlebot3**         | Robot simulation model in Gazebo ([details](./dockerfiles/turtlebot3)) | ✅ Available |
| **attestation-sidecar**   | ... ([details](./dockerfiles/attestation-sidecar)) | ✅ Available |
| **secaas**   | ... ([details](./dockerfiles/secaas)) | ✅ Available |

---

## Quick Setup

```bash
python3 create_agent_configurations.py --num-agents 10 --output ./config
```

Start the full experiment with all services:
- **4-node Private Ethereum-based blockchain** with [Hyperledger Besu](https://besu.hyperledger.org/private-networks) platform running [QBFT](https://besu.hyperledger.org/private-networks/how-to/configure/consensus/qbft) consensus algorithm
- **Gazebo** robot simulator
- **Robot(s)** with `turtlebot3-gazebo` and `attestation-sidecar`
- **Security-as-a-Service (SECaaS)**

```bash
./run_full_experiment_test.sh
```

> Note: This will start all required containers and start processing attestations (`AUTO_START=TRUE`)

Stop the experiment:

```bash
./stop_full_experiment.sh
```
---

## Blockchain Network Setup
1. Initialize the network
```bash
cd blockchain/quorum-test-network
./run.sh
```

Services exposed:
- [Block Explorer](http://localhost:25000/explorer/nodes)
- [Grafana](http://localhost:3000)

2. Deploy the [MasMutualAttestation.sol](./smart-contracts/contracts/MasMutualAttestation.sol) `smart contract`
```bash
./deploy_smart_contract.sh --rpc_url http://10.5.99.99:21001 --chain_id 1337
```

3. Remove the network
```bash
cd blockchain/quorum-test-network
./remove.sh
```

---

## Data collection (auto)

```bash
python3 run_experiments_and_collect_results.py --runs 5 --duration 180
```

---

## Data collection (manual)
1. Start containers:
```bash
docker compose up -d
```

2. Start docker stats data collection
```shell
curl -X POST localhost:6000/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{"containers": ["secaas","robot1-sidecar","robot2-sidecar","robot3-sidecar","robot4-sidecar"],
  "interval":1.0,"csv_dir":"/experiments/data/docker-stats/results","stdout":true}' | jq
```
> Check status:
```bash
curl localhost:6000/monitor/status | jq
```

3. Start attestation:
```bash
for p in 8080 8081 8082 8083 8084; do
  echo "Starting attestation on sidecar with port $p..."
  curl -X POST "http://localhost:${p}/start" | jq
done
```

4. Stop attestation:
```bash
for p in 8080 8081 8082 8083 8084; do
  echo "Stopping attestation on sidecar with port $p..."
  curl -X POST "http://localhost:${p}/stop" | jq
done
```

5. Stop docker stats data collection
```shell
curl -X POST "http://localhost:6000/monitor/stop" | jq
```

6. Stop workflow
```bash
docker compose down
```

---