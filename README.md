# Blockchain-based Mutual Remote Attestation for ROS2 Executables

**Author(s):** Adam Zahir and Mark Angoustures

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
| **hardhat**            | Development framework for compiling, testing, and deploying smart contracts on the Ethereum-based blockchain. ([Details](./dockerfiles/hardhat))         | ✅ Available |
| **gazebo-server**      | Headless Gazebo simulation server, shared by all robots. Handles physics, sensors, and simulation worlds. ([Details](./dockerfiles/gazebo-server)) | ✅ Available |
| **gazebo-gui**         | Optional graphical interface for viewing and interacting with the Gazebo simulation remotely through a web-based VNC desktop. Enabled with `start.sh --gui`. ([Details](./dockerfiles/gazebo-gui)) | ✅ Available |
| **robot-simulation**   | Runs the ROS 2 software stack for a single TurtleBot3 robot in simulation, including robot state publishing, navigation (Nav2), and mission and formation management. ([Details](./dockerfiles/robot-simulation)) | ✅ Available |
| **attestation-sidecar**   | Runs alongside each robot to prove its integrity and verify the integrity of other robots. ([Details](./dockerfiles/attestation-sidecar)) | ✅ Available |
| **secaas**   | Prepares each robot's environment for remote attestation. ([Details](./dockerfiles/secaas)) | ✅ Available |

---

## Automated Setup

1. Start the experimental scenario:

```bash
./start.sh --auto --ssp 20000 --cpu-limit 0.5 --iterq 1
```

The script automatically deploys the following components:

- **Private Ethereum blockchain:** A four-node private network running [Hyperledger Besu](https://besu.hyperledger.org/private-networks) with the [QBFT consensus algorithm](https://besu.hyperledger.org/private-networks/how-to/configure/consensus/qbft).
- **Gazebo:** Robot simulation environment.
- **Four TurtleBot3 robots:** Each robot runs a `robot-simulation` container and an `attestation-sidecar` container.
- **Security-as-a-Service (SECaaS):** Provides the infrastructure required for robot attestation.

The script also deploys the [`AttestationManagerV3.sol`](./smart-contracts/contracts/AttestationManagerV3.sol) smart contract to the blockchain.

Once deployed, the robots' attestation sidecars automatically start the attestation process (`AUTO_START=TRUE`). Each robot:

- **Attests itself** to prove its integrity.
- **Verifies other robots** by acting as an attestation verifier.

2. Stop the experimental scenario:

```bash
./stop.sh
```

---

## Manual Setup

### Blockchain Network

1. Initialize the blockchain:
```bash
cd blockchain/quorum-test-network
./run.sh
```

Services exposed:
- [Block Explorer](http://localhost:25000/explorer/nodes)
- [Grafana](http://localhost:3000)

2. Deploy the smart contract (`AttestationManagerLV`, `AttestationManagerV2` or `AttestationManagerV3`):

```bash
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337 --contract AttestationManagerLV
```

3. Remove the blockchain:

```bash
cd blockchain/quorum-test-network
./remove.sh
```

---

## Experiments

How to run each experiment for the final results: [README_EXPERIMENTS.md](./README_EXPERIMENTS.md).

---

## Agent configuration files

```bash
# Gazebo mode (config/)
python3 create_agent_config.py --num-agents 100 --config-output ./config \
    --cmd-name robot_state_publisher --text-section-size 42223 --offset 0

# Dummy mode (config-dummy/)
python3 create_agent_config.py --num-agents 100 --config-output ./config-dummy \
    --cmd-name dummy_publisher --text-section-size 175521 --offset 5744
```

> Note: To create blockchain agent credentials see [blockchain/quorum-test-network/extra](./blockchain/quorum-test-network/extra)


### Fix arp_cache: neighbor table overflow!

```bash
sudo sysctl -w net.ipv4.neigh.default.gc_thresh1=2048
sudo sysctl -w net.ipv4.neigh.default.gc_thresh2=4096
sudo sysctl -w net.ipv4.neigh.default.gc_thresh3=8192
```