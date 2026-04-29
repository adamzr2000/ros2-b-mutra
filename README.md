# Blockchain-based Mutual Remote Attestation for ROS2 Executables

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

1. Start the full experiment:

```bash
./start.sh --auto --contract standard
```

This will start all required containers and start processing attestations (`AUTO_START=TRUE`)
- **4-node Private Ethereum-based blockchain** with [Hyperledger Besu](https://besu.hyperledger.org/private-networks) platform running [QBFT](https://besu.hyperledger.org/private-networks/how-to/configure/consensus/qbft) consensus algorithm
- **Gazebo** robot simulator
- **4 Robot(s)** with `turtlebot3-gazebo` and `attestation-sidecar`
- **Security-as-a-Service (SECaaS)**

```bash
netcom@d-mutra:~/ros2-b-mutra$ ./start.sh -h
Usage: start.sh [--robots N] [--remote] [--auto] [--export] [--startup] [--wait-tx]
                        [--ssp N] [--cpu-limit X] [--contract standard|optimized] [--vrp N]

Options:
  --robots N     Number of robots to deploy (default: 4, max: 128).
  --remote       Remote deployment mode: d-mutra VM hosts only Besu+SECaaS+monitoring;
                 all robots+sidecars run on remote host(s) (remote1 up to 64 robots,
                 remote2 for 65–128 robots).
                 Default (local mode): everything runs on d-mutra.
  --auto         Set AUTO_START=TRUE
  --export       Set EXPORT_RESULTS=TRUE
  --wait-tx      Set WAIT_FOR_TX_CONFIRMATIONS=TRUE
  --startup      Set ONE_SHOT=TRUE
  --ssp N        Attestation interval in seconds — sets ATTESTATION_INTERVAL_MS=N*1000 (default: 20)
  --cpu-limit X  Sidecar CPU limit fraction — sets CPU_LIMIT=X in .env and compose files (default: 0.4)
  --contract standard|optimized  Smart contract variant (default: standard)
  --vrp N        Verifier Refreshing Period for optimized contract (default: 1)
  -h|--help      Show this help
```

2. Stop the experiment:

```bash
./stop.sh
```

---

## Blockchain Network Setup

1. Initialize the network:
```bash
cd blockchain/quorum-test-network
./run.sh
```

Services exposed:
- [Block Explorer](http://localhost:25000/explorer/nodes)
- [Grafana](http://localhost:3000)

2. Deploy a smart contract (`standard` or `optimized`):

```bash
# Standard (AttestationManager)
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337

# Optimized (AttestationManagerOptimized) — requires --vrp N
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337 --contract AttestationManagerOptimized --vrp 1
```

> **Note:** `start.sh` handles blockchain init and contract deployment automatically.

3. Remove the network:

```bash
cd blockchain/quorum-test-network
./remove.sh
```

---

## Data collection (attestation times + docker stats + blockchain stats)

1. Start the experimental setup:

```bash
# Local mode — everything runs on d-mutra VM (default)
./start.sh --robots 4 --export

# Remote mode — d-mutra VM hosts Besu+SECaaS+monitoring; robots+sidecars on remote1 host
./start.sh --robots 32 --export --remote
```

Use `--startup` for one-shot attestation mode, `--robots N` to scale (1–128).

2. Run attestation workflow in loop:

```bash
# Local — continuous mode
python3 run_experiments_and_collect_results.py --robots 4 --runs 5 --duration 120

# Local — startup (one-shot) mode
python3 run_experiments_and_collect_results.py --robots 4 --runs 10 --startup

# Remote — add --remote (must match --remote used in start.sh)
python3 run_experiments_and_collect_results.py --robots 32 --runs 5 --duration 120 --remote
python3 run_experiments_and_collect_results.py --robots 32 --runs 10 --startup --remote
```

`--robots N` must match the value used in `start.sh`.

3. Stop the experimental setup:

```bash
./stop.sh
```

---

## Data collection (robot application performance)

Measures per-message arrival timestamps for `/robotX/<topic>` with and without
attestation sidecars running. Results are stored under
`experiments/data/performance-benchmark/results/<topic>/<condition>/<mode>/run*.csv`
with columns `robot, topic, ros_stamp_ns, wall_stamp_ns`.

1. Start robots (no `--export` needed — sidecars will have export disabled):

```bash
./start.sh --robots 4
```

2. Start the topic collector, then run both conditions back-to-back:

```bash
# Default topic: /robotX/scan
COMPOSE_IGNORE_ORPHANS=1 \
  docker compose -f docker-compose.robots.yml \
                 -f docker-compose.benchmark-collector.yml \
                 up -d topic-collector

# Baseline — no sidecars running (attestation containers not started)
python3 run_benchmark.py --condition no_sidecar --runs 5 --topic scan

# With sidecars — continuous attestation mode
python3 run_benchmark.py --condition with_sidecar --mode continuous --runs 5 --topic scan

# With sidecars — startup (one-shot) attestation mode
python3 run_benchmark.py --condition with_sidecar --mode startup --runs 5 --topic scan
```

To benchmark a **different topic** (e.g. `/robotX/odom`), restart the collector with
`TOPIC_NAME` set and pass the matching `--topic` flag to the benchmark script:

```bash
COMPOSE_IGNORE_ORPHANS=1 TOPIC_NAME=/odom MSG_TYPE=nav_msgs.msg/Odometry \
  docker compose -f docker-compose.robots.yml \
                 -f docker-compose.benchmark-collector.yml \
                 up -d --force-recreate topic-collector

python3 run_benchmark.py --condition no_sidecar --runs 5 --topic odom
python3 run_benchmark.py --condition with_sidecar --mode continuous --runs 5 --topic odom
```

> `--robots N` in `start.sh` must match `N_ROBOTS` in the collector (default 4).

3. Stop the experimental setup:

```bash
./stop.sh
```

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