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

- `geth-node`: Based on [Go-Ethereum (Geth)](https://geth.ethereum.org/docs) software, serving as nodes within the private blockchain network. (detailed info [here](./dockerfiles/geth-node/)). ✅ Available

- `eth-netstats`: Dashboard for monitoring Geth nodes. (detailed info [here](./dockerfiles/eth-netstats/)). ✅ Available

- `truffle`: Development framework for Ethereum-based blockchain applications. It provides a suite of tools that allows developers to write, test, and deploy smart contracts. (detailed info [here](./dockerfiles/eth-netstats/)). ✅ Available

- `gazebo`: Robot simulation tool (detailed info [here](./dockerfiles/gazebo-vnc/)). ✅ Available

- `turtlebot3`: Robot simulation model in Gazebo (detailed info [here](./dockerfiles/turtlebot3/)). ✅ Available

---

## Quick Setup

Start the full experiment with all services:
- **Private blockchain** with `10 nodes`
- **Gazebo** robot simulator
- **Robot(s)** with `turtlebot3 ros2 simulation`, `sidecar-measurer`, and `sidecar-controller`
- **SECaaS** with `secaas-controller` and `secaas-wrapper`

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
2025-07-08 07:05:00,911 - INFO - [MainThread] Web3 initialized. Address: 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5
2025-07-08 07:05:00,916 - INFO - [MainThread] Connected to Ethereum node http://10.0.1.1:3334 | Geth Version: Geth/node1/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
=== Attestation chain is empty ===
2025-07-08 07:05:00,933 - INFO - [MainThread] Subscribed to attestation events...

2025-07-08 07:06:49,540 - INFO - [MainThread] Processing attestation '285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8'

2025-07-08 07:06:49,541 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:06:49,550 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:06:49,593 - INFO - [MainThread] Reference signature for agent with UUID '885f348810314e0cad017ffd15ed8ef4' sent

2025-07-08 07:06:49,606 - INFO - [MainThread] SECaaS is also the verifier

2025-07-08 07:06:54,668 - INFO - [MainThread] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979
  - Reference Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:06:54,676 - INFO - [MainThread] Comparing hashes...
2025-07-08 07:06:54,716 - INFO - [MainThread] Attestation closed (Result: ✅ SUCCESS)

=== Last 1 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:06:58,786 - INFO - [MainThread] Processing attestation 'fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff'

2025-07-08 07:06:58,786 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:06:58,794 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:06:58,828 - INFO - [MainThread] Reference signature for agent with UUID '5a1e3ef808df46a68aec1764fbae4161' sent

2025-07-08 07:06:58,838 - INFO - [MainThread] SECaaS is also the verifier

2025-07-08 07:07:04,912 - INFO - [MainThread] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166
  - Reference Signature: 0x988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166

2025-07-08 07:07:04,921 - INFO - [MainThread] Comparing hashes...
2025-07-08 07:07:04,963 - INFO - [MainThread] Attestation closed (Result: ✅ SUCCESS)

=== Last 2 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:07:09,010 - INFO - [MainThread] Processing attestation '54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4'

2025-07-08 07:07:09,010 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:07:09,018 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:07:09,057 - INFO - [MainThread] Reference signature for agent with UUID 'e3c12b0246db44409e90ebec84a105d4' sent

2025-07-08 07:07:09,068 - INFO - [MainThread] SECaaS is also the verifier

2025-07-08 07:07:14,127 - INFO - [MainThread] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd
  - Reference Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-08 07:07:14,135 - INFO - [MainThread] Comparing hashes...
2025-07-08 07:07:14,174 - INFO - [MainThread] Attestation closed (Result: ✅ SUCCESS)

=== Last 3 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
| 3 | 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958438 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:07:30,328 - INFO - [MainThread] Processing attestation '430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62'

2025-07-08 07:07:30,328 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:07:30,337 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:07:30,379 - INFO - [MainThread] Reference signature for agent with UUID '885f348810314e0cad017ffd15ed8ef4' sent

=== Last 4 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
| 3 | 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958438 |
| 4 | 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | ✅ SUCCESS | 1751958458 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:07:38,555 - INFO - [MainThread] Processing attestation '0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3'

2025-07-08 07:07:38,556 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:07:38,565 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:07:38,609 - INFO - [MainThread] Reference signature for agent with UUID '5a1e3ef808df46a68aec1764fbae4161' sent

=== Last 5 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
| 3 | 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958438 |
| 4 | 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | ✅ SUCCESS | 1751958458 |
| 5 | 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3 | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x10119812363DDf83693797700cD2565f68eb91b2 | ✅ SUCCESS | 1751958468 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:07:48,794 - INFO - [MainThread] Processing attestation 'b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212'

2025-07-08 07:07:48,794 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:07:48,803 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:07:48,841 - INFO - [MainThread] Reference signature for agent with UUID 'e3c12b0246db44409e90ebec84a105d4' sent

=== Last 6 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
| 3 | 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958438 |
| 4 | 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | ✅ SUCCESS | 1751958458 |
| 5 | 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3 | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x10119812363DDf83693797700cD2565f68eb91b2 | ✅ SUCCESS | 1751958468 |
| 6 | b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x10119812363DDf83693797700cD2565f68eb91b2 | ✅ SUCCESS | 1751958478 |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+

2025-07-08 07:08:08,149 - INFO - [MainThread] Processing attestation '4f3c73ea5a65dbebc9704ffc05619ad4fc822f48efb926aa1e167c58ae2df7c2'

2025-07-08 07:08:08,149 - INFO - [MainThread] Retrieving Prover UUID from SC...
2025-07-08 07:08:08,159 - INFO - [MainThread] Retrieving reference signature from DB...
2025-07-08 07:08:08,202 - INFO - [MainThread] Reference signature for agent with UUID '885f348810314e0cad017ffd15ed8ef4' sent

=== Last 7 Attestations ===
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| # | Attestation ID                                                   | Prover                                     | Verifier                                   | Result     | Timestamp  |
+---+------------------------------------------------------------------+--------------------------------------------+--------------------------------------------+------------+------------+
| 1 | 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958418 |
| 2 | fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958428 |
| 3 | 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x7342CDb3A535a664B3d6B4D2A5EAa292945D1cf5 | ✅ SUCCESS | 1751958438 |
| 4 | 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | ✅ SUCCESS | 1751958458 |
| 5 | 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3 | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | 0x10119812363DDf83693797700cD2565f68eb91b2 | ✅ SUCCESS | 1751958468 |
| 6 | b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212 | 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5 | 0x10119812363DDf83693797700cD2565f68eb91b2 | ✅ SUCCESS | 1751958478 |
| 7 | 4f3c73ea5a65dbebc9704ffc05619ad4fc822f48efb926aa1e167c58ae2df7c2 | 0x10119812363DDf83693797700cD2565f68eb91b2 | 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b | ✅ SUCCESS | 1751958498 |
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
2025-07-08 07:05:00,686 - INFO - [MainThread] Web3 initialized. Address: 0x10119812363DDf83693797700cD2565f68eb91b2
2025-07-08 07:05:00,693 - INFO - [MainThread] Connected to Ethereum node http://10.0.1.2:3335 | Geth Version: Geth/node2/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
2025-07-08 07:05:00,693 - INFO - [MainThread] Subscribed to attestation events...
2025-07-08 07:06:48,183 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:06:48,183 - INFO - [MainThread] Processing attestation: 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8
2025-07-08 07:06:48,200 - INFO - [Attest-285e] You are the Prover agent!

2025-07-08 07:06:48,236 - INFO - [Attest-285e] Fresh signature sent: 60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:06:48,236 - INFO - [Attest-285e] Waiting for attestation to complete...
2025-07-08 07:06:53,290 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:06:53,291 - INFO - [MainThread] Processing attestation: fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff
2025-07-08 07:06:53,293 - INFO - [MainThread] Processing attestation: 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4
2025-07-08 07:06:53,335 - WARNING - [Attest-fa90] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:53,337 - WARNING - [Attest-54df] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:59,364 - INFO - [Attest-285e] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8 | ✅ SUCCESS | 1751958418 |
+------------------------------------------------------------------+------------+------------+
2025-07-08 07:07:28,883 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:07:28,883 - INFO - [MainThread] Processing attestation: 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62
2025-07-08 07:07:28,913 - INFO - [Attest-430f] You are the Prover agent!

2025-07-08 07:07:28,953 - INFO - [Attest-430f] Fresh signature sent: 60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:07:28,953 - INFO - [Attest-430f] Waiting for attestation to complete...
2025-07-08 07:07:33,997 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:07:33,998 - INFO - [MainThread] Processing attestation: 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3
2025-07-08 07:07:33,999 - INFO - [MainThread] Processing attestation: b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212
2025-07-08 07:07:34,030 - INFO - [Attest-b460] You are the Verifier agent!

2025-07-08 07:07:34,030 - INFO - [Attest-b460] Waiting for responses from SECaaS and Prover...
2025-07-08 07:07:34,036 - INFO - [Attest-0fb9] You are the Verifier agent!

2025-07-08 07:07:34,036 - INFO - [Attest-0fb9] Waiting for responses from SECaaS and Prover...
2025-07-08 07:07:39,103 - INFO - [Attest-430f] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62 | ✅ SUCCESS | 1751958458 |
+------------------------------------------------------------------+------------+------------+
2025-07-08 07:07:44,190 - INFO - [Attest-0fb9] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166
  - Reference Signature: 0x988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166

2025-07-08 07:07:44,200 - INFO - [Attest-0fb9] Comparing hashes...
2025-07-08 07:07:44,258 - INFO - [Attest-0fb9] Attestation closed (Result: ✅ SUCCESS)

2025-07-08 07:07:54,288 - INFO - [Attest-b460] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd
  - Reference Signature: 0x37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-08 07:07:54,297 - INFO - [Attest-b460] Comparing hashes...
2025-07-08 07:07:54,332 - INFO - [Attest-b460] Attestation closed (Result: ✅ SUCCESS)

2025-07-08 07:08:08,700 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:08:08,701 - INFO - [MainThread] Processing attestation: 4f3c73ea5a65dbebc9704ffc05619ad4fc822f48efb926aa1e167c58ae2df7c2
2025-07-08 07:08:08,718 - INFO - [Attest-4f3c] You are the Prover agent!

2025-07-08 07:08:08,753 - INFO - [Attest-4f3c] Fresh signature sent: 60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:08:08,753 - INFO - [Attest-4f3c] Waiting for attestation to complete...
2025-07-08 07:08:13,816 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:08:13,816 - INFO - [MainThread] Processing attestation: 45379ebdb3498a9b8dd79532880b687c8462f90e408c8e865d22cebfea7fedd2
2025-07-08 07:08:13,842 - WARNING - [Attest-4537] You are not involved in this attestation. Ignoring...

2025-07-08 07:08:18,933 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:08:18,933 - INFO - [MainThread] Processing attestation: 497e625e7cadf1dc2537716cc3d55c3d73dd7abf44ddfba46891a078168ba2e7
2025-07-08 07:08:18,952 - WARNING - [Attest-497e] You are not involved in this attestation. Ignoring...

2025-07-08 07:08:19,906 - INFO - [Attest-4f3c] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 4f3c73ea5a65dbebc9704ffc05619ad4fc822f48efb926aa1e167c58ae2df7c2 | ✅ SUCCESS | 1751958498 |
+------------------------------------------------------------------+------------+------------+
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
2025-07-08 07:05:00,680 - INFO - [MainThread] Web3 initialized. Address: 0xc8d9C2f55b2C148cFD6384D164C5723D9EB992F5
2025-07-08 07:05:00,688 - INFO - [MainThread] Connected to Ethereum node http://10.0.1.3:3336 | Geth Version: Geth/node3/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
2025-07-08 07:05:00,688 - INFO - [MainThread] Subscribed to attestation events...
2025-07-08 07:06:48,162 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:06:48,162 - INFO - [MainThread] Processing attestation: 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8
2025-07-08 07:06:48,180 - WARNING - [Attest-285e] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:53,260 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:06:53,260 - INFO - [MainThread] Processing attestation: fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff
2025-07-08 07:06:53,261 - INFO - [MainThread] Processing attestation: 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4
2025-07-08 07:06:53,297 - WARNING - [Attest-fa90] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:53,302 - INFO - [Attest-54df] You are the Prover agent!

2025-07-08 07:06:53,345 - INFO - [Attest-54df] Fresh signature sent: 37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-08 07:06:53,345 - INFO - [Attest-54df] Waiting for attestation to complete...
2025-07-08 07:07:19,636 - INFO - [Attest-54df] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4 | ✅ SUCCESS | 1751958438 |
+------------------------------------------------------------------+------------+------------+
2025-07-08 07:07:28,882 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:07:28,882 - INFO - [MainThread] Processing attestation: 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62
2025-07-08 07:07:28,898 - INFO - [Attest-430f] You are the Verifier agent!

2025-07-08 07:07:28,898 - INFO - [Attest-430f] Waiting for responses from SECaaS and Prover...
2025-07-08 07:07:34,009 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:07:34,009 - INFO - [MainThread] Processing attestation: 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3
2025-07-08 07:07:34,010 - INFO - [MainThread] Processing attestation: b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212
2025-07-08 07:07:34,057 - WARNING - [Attest-0fb9] You are not involved in this attestation. Ignoring...

2025-07-08 07:07:34,060 - INFO - [Attest-b460] You are the Prover agent!

2025-07-08 07:07:34,103 - INFO - [Attest-b460] Fresh signature sent: 37c0e3f7c3e2bf4bbdfd787eb5952db4edd5f663ce5e6bb745e22442686deddd

2025-07-08 07:07:34,103 - INFO - [Attest-b460] Waiting for attestation to complete...
2025-07-08 07:07:34,986 - INFO - [Attest-430f] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979
  - Reference Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:07:34,995 - INFO - [Attest-430f] Comparing hashes...
2025-07-08 07:07:35,055 - INFO - [Attest-430f] Attestation closed (Result: ✅ SUCCESS)

2025-07-08 07:07:59,396 - INFO - [Attest-b460] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212 | ✅ SUCCESS | 1751958478 |
+------------------------------------------------------------------+------------+------------+
```

### sidecar-controller-robot3 logs

```bash
netcom@d-mutra:~$ docker logs -f sidecar-controller-robot3
```

```plaintext
Selected Execution Parameters:
  - Participant       : agent
  - Export results    : Disabled
  - Bootstrap mode    : Disabled
  - Fail mode         : Disabled
2025-07-08 07:05:01,057 - INFO - [MainThread] Web3 initialized. Address: 0xCd5aE667b462695F3f54A613aE43D7D00c9FA25b
2025-07-08 07:05:01,061 - INFO - [MainThread] Connected to Ethereum node http://10.0.1.4:3337 | Geth Version: Geth/node4/v1.13.15-stable-c5ba367e/linux-amd64/go1.21.6
2025-07-08 07:05:01,061 - INFO - [MainThread] Subscribed to attestation events...
2025-07-08 07:06:48,508 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:06:48,509 - INFO - [MainThread] Processing attestation: 285ee89fa82548bf348c9f7eabe0459d127b04080f1318dbf9156e708df6a4b8
2025-07-08 07:06:48,529 - WARNING - [Attest-285e] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:53,597 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:06:53,597 - INFO - [MainThread] Processing attestation: fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff
2025-07-08 07:06:53,598 - INFO - [MainThread] Processing attestation: 54df5e093ca159e9279ff2391d52e341a818ece8abc3496705be9d78b9b52ab4
2025-07-08 07:06:53,631 - INFO - [Attest-fa90] You are the Prover agent!

2025-07-08 07:06:53,637 - WARNING - [Attest-54df] You are not involved in this attestation. Ignoring...

2025-07-08 07:06:53,675 - INFO - [Attest-fa90] Fresh signature sent: 988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166

2025-07-08 07:06:53,675 - INFO - [Attest-fa90] Waiting for attestation to complete...
2025-07-08 07:07:09,850 - INFO - [Attest-fa90] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| fa90134e2647e6944e58d110f8008933736e245460394ff27a829824668ed8ff | ✅ SUCCESS | 1751958428 |
+------------------------------------------------------------------+------------+------------+
2025-07-08 07:07:28,179 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:07:28,179 - INFO - [MainThread] Processing attestation: 430ffaae7188d11e7f1c1755e05c5a696796be8387800fe5770c342383121f62
2025-07-08 07:07:28,196 - WARNING - [Attest-430f] You are not involved in this attestation. Ignoring...

2025-07-08 07:07:33,286 - INFO - [MainThread] Found 2 new attestation(s)…

2025-07-08 07:07:33,286 - INFO - [MainThread] Processing attestation: 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3
2025-07-08 07:07:33,286 - INFO - [MainThread] Processing attestation: b4604dcc46c6bb48c87ddef2c4627d794e8b4df2265312989675454acf4a2212
2025-07-08 07:07:33,327 - INFO - [Attest-0fb9] You are the Prover agent!

2025-07-08 07:07:33,333 - WARNING - [Attest-b460] You are not involved in this attestation. Ignoring...

2025-07-08 07:07:33,369 - INFO - [Attest-0fb9] Fresh signature sent: 988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166

2025-07-08 07:07:33,369 - INFO - [Attest-0fb9] Waiting for attestation to complete...
2025-07-08 07:07:49,548 - INFO - [Attest-0fb9] Attestation state: Closed
+------------------------------------------------------------------+------------+------------+
| Attestation ID                                                   | Result     | Timestamp  |
+------------------------------------------------------------------+------------+------------+
| 0fb9ef0fd50b7cbadae754ea13ce015e109f657430d9228453f9f89276ccefd3 | ✅ SUCCESS | 1751958468 |
+------------------------------------------------------------------+------------+------------+
2025-07-08 07:08:08,950 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:08:08,950 - INFO - [MainThread] Processing attestation: 4f3c73ea5a65dbebc9704ffc05619ad4fc822f48efb926aa1e167c58ae2df7c2
2025-07-08 07:08:08,965 - INFO - [Attest-4f3c] You are the Verifier agent!

2025-07-08 07:08:08,965 - INFO - [Attest-4f3c] Waiting for responses from SECaaS and Prover...
2025-07-08 07:08:13,073 - INFO - [MainThread] Found 1 new attestation(s)…

2025-07-08 07:08:13,073 - INFO - [MainThread] Processing attestation: 45379ebdb3498a9b8dd79532880b687c8462f90e408c8e865d22cebfea7fedd2
2025-07-08 07:08:13,091 - INFO - [Attest-4537] You are the Prover agent!

2025-07-08 07:08:13,132 - INFO - [Attest-4537] Fresh signature sent: 988f5d25a557b736d63d0176e4ddbcba93ac741139d25e9da1da7bd670935166

2025-07-08 07:08:13,132 - INFO - [Attest-4537] Waiting for attestation to complete...
2025-07-08 07:08:15,063 - INFO - [Attest-4f3c] Attestation state: ReadyForEvaluation
Attestation measurements:
  - Fresh Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979
  - Reference Signature: 0x60c550260bcd69de5404eb60e744ab1911ca324a6b8d08cf9b8115c7ce9e2979

2025-07-08 07:08:15,071 - INFO - [Attest-4f3c] Comparing hashes...
2025-07-08 07:08:15,113 - INFO - [Attest-4f3c] Attestation closed (Result: ✅ SUCCESS)
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
