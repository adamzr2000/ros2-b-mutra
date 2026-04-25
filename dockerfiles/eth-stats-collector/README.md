# Ethereum Block Stats Collector

Per-block metrics collector for a running Ethereum-compatible node (tested with Hyperledger Besu / QBFT).

## Metrics Description

Each row in the CSV corresponds to one block observed on-chain.

- **collected_at_iso** — Wall-clock timestamp (UTC ISO-8601) when the block was fetched by the collector.
- **block_number** — Block height.
- **block_timestamp** — Unix epoch timestamp embedded in the block header (seconds).
- **block_time_s** — Seconds elapsed since the previous block (`block_timestamp[n] - block_timestamp[n-1]`). Empty for the first collected block.
- **tx_count** — Number of transactions included in the block.
- **gas_used** — Total gas consumed by all transactions in the block.
- **gas_limit** — Maximum gas allowed per block as configured on the network.
- **gas_used_pct** — `100 × gas_used / gas_limit` (%).
- **size_bytes** — Full RLP-encoded block size in bytes (header + transactions + uncles). Empty blocks on the test network are ~838 bytes; the delta above that baseline reflects transaction payload.
- **extra_data_bytes** — Length of the raw `extraData` field in bytes. Always 0 when the POA middleware is active (QBFT encodes validator seals here; the middleware strips it before the block is returned).
- **avg_gas_per_tx** — `gas_used / tx_count`. 0 when the block is empty.

## Notes

- Connectivity to the Besu node is via `host.docker.internal` (resolved through the `extra_hosts` entry in the compose file), so the container does not need to join the blockchain's internal Docker network.
- The POA middleware (`ExtraDataToPOAMiddleware`) is injected automatically to handle the oversized `extraData` field produced by QBFT.
- Auto-naming: if no `csv_name` is provided, files are named `blockchain-run1.csv`, `blockchain-run2.csv`, etc., incrementing from the highest existing index.

## Usage

Check health:
```shell
curl localhost:7000/ | jq
```

Start monitoring (auto-named CSV):
```shell
curl -X POST localhost:7000/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{
    "rpc_url": "http://host.docker.internal:21001",
    "poll_interval": 1.0,
    "csv_dir": "/experiments/data/blockchain-stats/results"
  }' | jq
```

Start monitoring (explicit CSV name):
```shell
curl -X POST localhost:7000/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{
    "rpc_url": "http://host.docker.internal:21001",
    "poll_interval": 1.0,
    "csv_dir": "/experiments/data/blockchain-stats/results",
    "csv_name": "run-4robots-continuous"
  }' | jq
```

Check status:
```shell
curl localhost:7000/monitor/status | jq
```

Get last collected block sample:
```shell
curl localhost:7000/monitor/last | jq
```

Stop monitoring:
```shell
curl -X POST localhost:7000/monitor/stop | jq
```
