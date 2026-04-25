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

1. Create agent configurations:

```bash
# Gazebo mode (config/)
python3 create_agent_config.py --num-agents 100 --config-output ./config \
    --cmd-name robot_state_publisher --text-section-size 42223 --offset 0

# Dummy mode (config-dummy/)
python3 create_agent_config.py --num-agents 100 --config-output ./config-dummy \
    --cmd-name dummy_publisher --text-section-size 175521 --offset 5744
```

> Note: To create blockchain agent credentials see [blockchain/quorum-test-network/extra](./blockchain/quorum-test-network/extra)

2. Start the full experiment:

```bash
./start.sh --auto
```

This will start all required containers and start processing attestations (`AUTO_START=TRUE`)
- **4-node Private Ethereum-based blockchain** with [Hyperledger Besu](https://besu.hyperledger.org/private-networks) platform running [QBFT](https://besu.hyperledger.org/private-networks/how-to/configure/consensus/qbft) consensus algorithm
- **Gazebo** robot simulator
- **Robot(s)** with `turtlebot3-gazebo` and `attestation-sidecar`
- **Security-as-a-Service (SECaaS)**

```bash
./start.sh -h
Usage: start.sh [--auto] [--export] [--startup]

Options:
  --auto      Set AUTO_START=TRUE
  --export    Set EXPORT_RESULTS=TRUE
  --startup   Set ONE_SHOT=TRUE (only for startup attestation process)
  --wait-tx   Set WAIT_FOR_TX_CONFIRMATIONS=TRUE
  -h|--help   Show this help
```

3. Stop the experiment:

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

2. Deploy the [MasMutualAttestation.sol](./smart-contracts/contracts/MasMutualAttestation.sol) `smart contract`:

```bash
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337
```

3. Remove the network:

```bash
cd blockchain/quorum-test-network
./remove.sh
```

---

## Data collection (attestation times + docker stats)

1. Start the experimental setup (default: 4 robots):

```bash
./start.sh --robots 4 --export
```

Use `--startup` for one-shot mode, `--robots N` for scale scenarios (1–100).

2. Run attestation workflow in loop:

```bash
# Continuous mode
python3 run_experiments_and_collect_results.py --robots 4 --runs 5 --duration 120

# Startup (one-shot) mode
python3 run_experiments_and_collect_results.py --robots 4 --runs 10 --startup
```

`--robots N` must match the value used in `start.sh`.

3. Stop the experimental setup:

```bash
./stop.sh
```

---

## Data collection (ros2 topic hz)

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


<!-- ## Data collection (manual)
1. Start containers:
```bash
./start.sh
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
for p in 8000 8001 8002 8003 8004; do
  echo "Starting attestation on sidecar with port $p..."
  curl -X POST "http://localhost:${p}/start" | jq
done
```

4. Stop attestation:
```bash
for p in 8000 8001 8002 8003 8004; do
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
./stop.sh
``` -->
