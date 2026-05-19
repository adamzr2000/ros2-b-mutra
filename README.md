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
./start.sh --auto --contract rr --ssp 20000 --cpu-limit 0.5 --iterq 1
```

This will start all required containers, deploy the [AttestationManagerRR.sol](./smart-contracts/contracts/AttestationManagerRR.sol) or [AttestationManagerRR.sol](./smart-contracts/contracts/AttestationManagerLV.sol.sol)
and start processing attestations (`AUTO_START=TRUE`)
- **4-node Private Ethereum-based blockchain** with [Hyperledger Besu](https://besu.hyperledger.org/private-networks) platform running [QBFT](https://besu.hyperledger.org/private-networks/how-to/configure/consensus/qbft) consensus algorithm
- **Gazebo** robot simulator
- **4 Robot(s)**, each including `turtlebot3-gazebo` and `attestation-sidecar` container images
- **Security-as-a-Service (SECaaS)**

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

2. Deploy a smart contract (`rr` or `lv`):

```bash
# Round-robin (AttestationManagerRR)
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337 --contract AttestationManagerRR

# Last-verifier (AttestationManagerLV)
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337 --contract AttestationManagerLV
```

> **Note:** [start.sh](./start.sh) handles blockchain init and contract deployment automatically.

3. Remove the network:

```bash
cd blockchain/quorum-test-network
./remove.sh
```

---

## Data collection (attestation times + docker stats)

1. Start the experimental setup:

```bash
# Local mode (default)
./start.sh --robots 4 --export --contract rr --ssp 20000 --cpu-limit 0.4 --iterq 1

# Remote mode — Blockchain+SECaaS+Monitoring on local host; Robots+Sidecars+Monitoring on remote1 host
./start.sh --robots 32 --export --remote --contract rr --ssp 20000 --cpu-limit 0.4 --iterq 1
```

Use `--startup` for one-shot attestation mode, `--robots N` to scale (1–128).

2. Run attestation workflow in loop:

```bash
# Local — continuous mode (--variant must match contract deployed by start.sh: rr or lv)
python3 run_experiments_and_collect_results.py --robots 4 --runs 5 --duration 120 --variant lv

# Local — startup (one-shot) mode (no variant needed)
python3 run_experiments_and_collect_results.py --robots 4 --runs 10 --startup

# Remote — add --remote (must match --remote used in start.sh)
python3 run_experiments_and_collect_results.py --robots 32 --runs 5 --duration 120 --variant lv --remote
python3 run_experiments_and_collect_results.py --robots 32 --runs 10 --startup --remote
```

`--robots N` must match the value used in `start.sh`. `--variant` sets the results subdirectory (`rr` or `lv`) and must match the contract deployed.

3. Stop the experimental setup:

```bash
./stop.sh
```

---

## Data collection (blockchain stats)

Measures on-chain activity (transactions per block, gas, block time) while robots perform
continuous attestation. Three sweeps are collected:

- **Idle** — no attestation, baseline chain activity
- **N sweep** — fixed SSP and ITERQ, increasing number of robots
- **ITERQ sweep** — fixed N (largest), fixed SSP, increasing ITERQ

Results: `experiments/data/blockchain-stats/results/`
Tagged: `N{N}/blockchain-SSP{N}ms-ITERQ{N}-run{N}.csv`. Run index is auto-incremented.

1. Start the experimental setup:

```bash
# Local
./start.sh --robots <N> --contract rr --ssp 20000 --iterq 1

# Remote
./start.sh --robots <N> --remote --contract rr --ssp 20000 --iterq 1
```

SSP and ITERQ set here are overridden at runtime by the collection script, so these
values only affect what the containers start with before the first `/config` push.

2. Collect idle baseline

No robots active — just the chain running:

```bash
python3 collect_blockchain_stats_idle.py --duration 300
```

Results go to `experiments/data/blockchain-stats/results/idle/`.

3. N sweep (fixed SSP, fixed ITERQ)

```bash
python3 run_experiments_and_collect_blockchain_stats.py --robots <N> --ssp 20000 --iterq 1 --duration 300
```

Restart the the experimental setup for each N

Add `--remote` to both `start.sh` and the collection script when robots are on a remote host.

4. ITERQ sweep (largest N, fixed SSP)

Keep the experimental setup running at the largest N from Step 3.

```bash
python3 run_experiments_and_collect_blockchain_stats.py --robots 64 --ssp 20000 --iterq 2  --duration 300
python3 run_experiments_and_collect_blockchain_stats.py --robots 64 --ssp 20000 --iterq 4  --duration 300
python3 run_experiments_and_collect_blockchain_stats.py --robots 64 --ssp 20000 --iterq 8  --duration 300
```

Add `--remote` to both `start.sh` and the collection script when robots are on a remote host.

5. Stop

```bash
./stop.sh
```

---

## Data collection (robot app performance)

Measures wall-clock inter-arrival intervals of `/robotX/tf` with and without attestation
sidecars. The topic is published by `robot_state_publisher` — the exact process being
attested — so any sidecar-induced CPU interference shows up directly as delivery jitter.

Results: `experiments/data/robot-stats/results/tf/<condition>/<mode>/run*.csv`
Tagged: `SSP{N}ms-ITERQ{N}-cpu{N|NC}-run{N}.csv` (`NC` = no CPU cap).

Two sweeps are collected. SSP is pushed to sidecars at runtime via `/config`; the CPU
cap is a cgroup set at container start and requires a stack restart to change.

- **SSP sweep** — no CPU cap (`--no-cpu-limit`), SSP ∈ {5000, 1000, 500, 100, 10} ms
- **CPU sweep** — fixed SSP = 10 ms (most aggressive), CPU ∈ {0.5, 0.25, 0.1} cores

1. Start the experimental setup:

```bash
# SSP sweep: no CPU cap (tagged cpuNC)
./start.sh --robots 4 --contract rr --no-cpu-limit --no-wait-result

# CPU sweep: restart once per CPU limit value
./start.sh --robots 4 --contract rr --ssp 10 --cpu-limit <0.5|0.25|0.1> --no-wait-result
```

2. Start the topic collector

```bash
COMPOSE_IGNORE_ORPHANS=1 TOPIC_NAME=/tf MSG_TYPE=tf2_msgs.msg/TFMessage \
  docker compose -f docker-compose.robots.yml \
                 -f docker-compose.benchmark-collector.yml \
                 up -d --force-recreate topic-collector
```

3. Run conditions

```bash
# Baseline (run once, no sidecars)
python3 run_robot_benchmark.py --condition no_sidecar --topic tf --duration 300

# SSP sweep (no CPU cap)
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 5000
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 1000
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 500
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 100
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 10

# CPU sweep — stop stack, restart with new cpu-limit, restart collector, then run:
docker compose -f docker-compose.benchmark-collector.yml down && ./stop.sh
./start.sh --robots 4 --contract rr --ssp 10 --cpu-limit <X> --no-wait-result
# (restart collector as above)
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 10 --cpu-limit 0.5
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 10 --cpu-limit 0.25
python3 run_robot_benchmark.py --condition with_sidecar --topic tf --duration 300 --ssp 10 --cpu-limit 0.1
```

### Step 4 — Stop

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