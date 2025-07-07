# Blockchain-based Mutual Remote Attestation (B-MUTRA) for ROS2 Executables

**Author(s):** Adam Zahir Rodriguez and Mark Angoustures

## Installation
Before getting started, make sure you have the [Docker](https://docs.docker.com/engine/install/ubuntu/) and [Docker Compose](https://docs.docker.com/compose/install/linux/) installed on your system.

1. Clone the repository:
```bash
git clone git@github.com:adamzr2000/ros2-b-mutra.git
```

2. Build Docker Images:
Go to the [dockerfiles](./dockerfiles) directory and run the `./build.sh` scripts for each image:

- `dlt-node`: Based on [Go-Ethereum (Geth)](https://geth.ethereum.org/docs) software, serving as nodes within the private blockchain network. (detailed info [here](./dockerfiles/dlt-node/)). ✅ Available

- `eth-netstats`: Dashboard for monitoring Geth nodes. (detailed info [here](./dockerfiles/eth-netstats/)). ✅ Available

- `truffle`: Development framework for Ethereum-based blockchain applications. It provides a suite of tools that allows developers to write, test, and deploy smart contracts. (detailed info [here](./dockerfiles/eth-netstats/)). ✅ Available

- `gazebo`: Robot simulation tool (detailed info [here](./dockerfiles/gazebo-vnc/)). ✅ Available

- `turtlebot3`: Robot simulation model in Gazebo (detailed info [here](./dockerfiles/turtlebot3/)). ✅ Available

---

## Quick Setup

Start the full experiment with all services (blockchain network + gazebo + simulated robots + secaas controller + secaas wrapper + sidecar robot controllers + sidecar robot measurers):

```bash
./run_full_experiment.sh
```

> Note: This will start all required containers and start processing attestations.

### secaas logs
```bash
netcom@d-mutra:~$ docker logs -f secaas
```

```plaintext
Selected Execution Parameters:
  - Participant       : secaas
  - Export results    : Disabled
  - Bootstrap mode    : Disabled
  - Fail mode         : Disabled
2025-07-07 11:17:20,420 - INFO - Web3 initialized. Address: 0x5174423B3aABceD1833d612d99c38A58dc639E49
2025-07-07 11:17:20,423 - INFO - Connected to Ethereum node ws://10.0.1.1:3334 | Geth Version: Geth/node1/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
=== Attestation chain is empty ===
2025-07-07 11:17:20,434 - INFO - Subscribed to attestation events...

2025-07-07 11:19:02,831 - INFO - Processing attestation: 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6

2025-07-07 11:19:02,832 - INFO - Retrieving Prover UUID from SC...
2025-07-07 11:19:02,838 - INFO - Retrieving reference signature from DB...
2025-07-07 11:19:02,872 - INFO - Reference signature for agent with UUID '885f348810314e0cad017ffd15ed8ef4' sent

2025-07-07 11:19:02,880 - INFO - SECaaS is also the verifier

2025-07-07 11:19:08,930 - INFO - Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979
  - Reference Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-07 11:19:08,951 - INFO - Comparing hashes...
2025-07-07 11:19:08,982 - INFO - Attestation closed (Result: ✅ SUCCESS)

=== Last 1 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887152 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-07 11:19:13,013 - INFO - Processing attestation: 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8

2025-07-07 11:19:13,013 - INFO - Retrieving Prover UUID from SC...
2025-07-07 11:19:13,021 - INFO - Retrieving reference signature from DB...
2025-07-07 11:19:13,051 - INFO - Reference signature for agent with UUID 'e3c12b0246db44409e90ebec84a105d4' sent

2025-07-07 11:19:13,058 - INFO - SECaaS is also the verifier

2025-07-07 11:19:18,111 - INFO - Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd
  - Reference Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-07 11:19:18,128 - INFO - Comparing hashes...
2025-07-07 11:19:18,156 - INFO - Attestation closed (Result: ✅ SUCCESS)

=== Last 2 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887152 |
| 2 | 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887162 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-07 11:19:43,307 - INFO - Processing attestation: f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010

2025-07-07 11:19:43,307 - INFO - Retrieving Prover UUID from SC...
2025-07-07 11:19:43,315 - INFO - Retrieving reference signature from DB...
2025-07-07 11:19:43,344 - INFO - Reference signature for agent with UUID '885f348810314e0cad017ffd15ed8ef4' sent

=== Last 3 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887152 |
| 2 | 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887162 |
| 3 | f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | ✅ SUCCESS | 1751887192 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-07 11:19:52,450 - INFO - Processing attestation: 2b7cb4c1d5ab781ef5e393ab3e456dc0c664c63698aa62570eeff23766516773

2025-07-07 11:19:52,450 - INFO - Retrieving Prover UUID from SC...
2025-07-07 11:19:52,458 - INFO - Retrieving reference signature from DB...
2025-07-07 11:19:52,489 - INFO - Reference signature for agent with UUID 'e3c12b0246db44409e90ebec84a105d4' sent

=== Last 4 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887152 |
| 2 | 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | 0x5174423B3aABceD1833d612d99c38A58dc639E49 | ✅ SUCCESS | 1751887162 |
| 3 | f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | ✅ SUCCESS | 1751887192 |
| 4 | 2b7cb4c1d5ab781ef5e393ab3e456dc0c664c63698aa62570eeff23766516773 | 0xB407B1C625290695467dAE9507E3A1a32F8f5773 | 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86 | ✅ SUCCESS | 1751887202 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
```

### sidecar-controller-robot1 logs

```bash
netcom@d-mutra:~$ docker logs -f sidecar-controller-robot1
```

```plaintext
Selected Execution Parameters:
  - Participant       : agent
  - Export results    : Enabled
  - Bootstrap mode    : Disabled
  - Fail mode         : Disabled
2025-07-07 11:17:20,729 - INFO - Web3 initialized. Address: 0xd95fFe906eD576dA6f178CC7cff2Cf3784e4CB86
2025-07-07 11:17:20,732 - INFO - Connected to Ethereum node ws://10.0.1.2:3335 | Geth Version: Geth/node2/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
2025-07-07 11:17:20,733 - INFO - Subscribed to attestation events...
2025-07-07 11:19:03,117 - INFO - Found 2 new attestation(s)…

2025-07-07 11:19:03,117 - INFO - Processing attestation: 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6
2025-07-07 11:19:03,130 - INFO - You are the Prover agent!

2025-07-07 11:19:03,159 - INFO - Fresh signature sent: 60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-07 11:19:03,159 - INFO - Waiting for attestation to complete...
2025-07-07 11:19:13,239 - INFO - Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6 | ✅ SUCCESS | 1751887152 |
+------------------------------------------------------------------+------------+------------+
2025-07-07 11:19:13,247 - INFO - Attestation duration (Prover perspective): 10.12 seconds

2025-07-07 11:19:13,247 - INFO - Processing attestation: 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8
2025-07-07 11:19:13,260 - WARNING - You are not involved in this attestation. Ignoring...

2025-07-07 11:19:43,398 - INFO - Found 2 new attestation(s)…

2025-07-07 11:19:43,398 - INFO - Processing attestation: f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010
2025-07-07 11:19:43,412 - INFO - You are the Prover agent!

2025-07-07 11:19:43,440 - INFO - Fresh signature sent: 60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-07 11:19:43,440 - INFO - Waiting for attestation to complete...
2025-07-07 11:19:54,529 - INFO - Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010 | ✅ SUCCESS | 1751887192 |
+------------------------------------------------------------------+------------+------------+
2025-07-07 11:19:54,537 - INFO - Attestation duration (Prover perspective): 11.12 seconds

2025-07-07 11:19:54,537 - INFO - Processing attestation: 2b7cb4c1d5ab781ef5e393ab3e456dc0c664c63698aa62570eeff23766516773
2025-07-07 11:19:54,550 - INFO - You are the Verifier agent!

2025-07-07 11:19:54,550 - INFO - Waiting for responses from SECaaS and Prover...
2025-07-07 11:19:58,590 - INFO - Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd
  - Reference Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-07 11:19:58,597 - INFO - Comparing hashes...
2025-07-07 11:19:58,629 - INFO - Attestation closed (Result: ✅ SUCCESS)

2025-07-07 11:19:58,630 - INFO - Attestation duration (Verifier perspective): 4.08 seconds
```

### sidecar-controller-robot2 logs

```bash
netcom@d-mutra:~$ docker logs -f sidecar-controller-robot2
```

```plaintext
Selected Execution Parameters:
  - Participant       : agent
  - Export results    : Disabled
  - Bootstrap mode    : Disabled
  - Fail mode         : Disabled
2025-07-07 11:17:20,811 - INFO - Web3 initialized. Address: 0xB407B1C625290695467dAE9507E3A1a32F8f5773
2025-07-07 11:17:20,815 - INFO - Connected to Ethereum node ws://10.0.1.3:3336 | Geth Version: Geth/node3/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
2025-07-07 11:17:20,815 - INFO - Subscribed to attestation events...
2025-07-07 11:19:03,205 - INFO - Found 2 new attestation(s)…

2025-07-07 11:19:03,205 - INFO - Processing attestation: 46b4b9e492576772c479ab9744eab8ce9bdb892f2e61b88e5ee535321a3594a6
2025-07-07 11:19:03,219 - WARNING - You are not involved in this attestation. Ignoring...

2025-07-07 11:19:06,219 - INFO - Processing attestation: 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8
2025-07-07 11:19:06,236 - INFO - You are the Prover agent!

2025-07-07 11:19:06,268 - INFO - Fresh signature sent: 37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-07 11:19:06,268 - INFO - Waiting for attestation to complete...
2025-07-07 11:19:23,406 - INFO - Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 149447fa0caffdb5bedacc669994e9e976d2f62aee075720ab43be0e5f947ae8 | ✅ SUCCESS | 1751887162 |
+------------------------------------------------------------------+------------+------------+
2025-07-07 11:19:23,413 - INFO - Attestation duration (Prover perspective): 17.18 seconds

2025-07-07 11:19:44,527 - INFO - Found 2 new attestation(s)…

2025-07-07 11:19:44,528 - INFO - Processing attestation: f79e8ea264c29b4d37a3e0fffacea60f3ea614b48dc035b88c882a58a6be1010
2025-07-07 11:19:44,542 - INFO - You are the Verifier agent!

2025-07-07 11:19:44,542 - INFO - Waiting for responses from SECaaS and Prover...
2025-07-07 11:19:48,579 - INFO - Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979
  - Reference Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-07 11:19:48,586 - INFO - Comparing hashes...
2025-07-07 11:19:48,620 - INFO - Attestation closed (Result: ✅ SUCCESS)

2025-07-07 11:19:48,620 - INFO - Attestation duration (Verifier perspective): 4.08 seconds

2025-07-07 11:19:51,624 - INFO - Processing attestation: 2b7cb4c1d5ab781ef5e393ab3e456dc0c664c63698aa62570eeff23766516773
2025-07-07 11:19:51,637 - INFO - You are the Prover agent!

2025-07-07 11:19:51,670 - INFO - Fresh signature sent: 37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-07 11:19:51,670 - INFO - Waiting for attestation to complete...
2025-07-07 11:20:03,772 - INFO - Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 2b7cb4c1d5ab781ef5e393ab3e456dc0c664c63698aa62570eeff23766516773 | ✅ SUCCESS | 1751887202 |
+------------------------------------------------------------------+------------+------------+
2025-07-07 11:20:03,778 - INFO - Attestation duration (Prover perspective): 12.14 seconds
```

---

```bash
./stop_full_experiment.sh
```
---

## Blockchain Network Setup
1. Initialize the network
```bash
cd dlt-network/geth-poa
sudo ./start_geth_poa_network.sh
```

Access the `eth-netsats` web interface for additional information at [http://10.5.99.99:3000](http://10.5.99.99:3000)

2. Deploy the [MasMutualAttestation](./smart-contracts/contracts/MasMutualAttestation.sol) smart contract
```bash
./deploy_smart_contract.sh --node-ip 10.0.1.1 --ws-port 3334 
```

3. Stop the network
```bash
cd dlt-network/geth-poa
sudo ./stop_geth_poa_network.sh
```

<!-- ## Usage

1. Start the experimental scenario
```bash
docker compose up -d
```

2. Run blockchain-based attestation on SECaaS
Execute the mutual attestation script inside the `secaas` container:

```bash
docker exec -it secaas bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant secaas"
```

3. Run blockchain-based attestation on Robots
Execute the mutual attestation script for each `robot` container:

```bash
docker exec -it robot1 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
```

```bash
docker exec -it robot2 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
```

```bash
docker exec -it robot3 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap"
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
docker exec -it secaas bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant secaas --reset-chain"
```

### Retrieve blockchain transaction receipt
```bash
docker exec -it secaas bash -c "source ~/.profile && cd ~/scripts && python3 get_tx_receipt.py --eth-node-url ws://10.0.1.1:3334 --tx-hash <hash>"
```

### Remove registered agent from the smart contract
```bash
docker exec -it robot1 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --remove-agent"
```

### Simulate failed attestation
```bash
docker exec -it robot1 bash -c "source ~/.profile && cd ~/scripts && python3 mas_mutual_attestation.py --participant agent --bootstrap --fail-attestation"
```
 -->
