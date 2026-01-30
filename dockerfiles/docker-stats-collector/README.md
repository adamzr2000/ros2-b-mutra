# Docker Stats Monitoring

Per-container resource monitoring based on Docker statistics and host-level fallbacks.

## Metrics Description

Each sample corresponds to a measurement window ending at the reported timestamp and is collected per monitored container.

### CPU Metrics

- **cpu_percent**  
  Container CPU utilization (%) computed from Docker stats CPU time deltas  
  (`cpu_stats` vs `precpu_stats`), normalized by the number of online CPUs.  
  Represents the fraction of host CPU time consumed by the container during the most recent reporting window.

### Memory Metrics

- **mem_mb**  
  Container memory usage (MB) computed as **memory usage minus page cache**  
  (`memory_stats.usage - memory_stats.stats.cache`).  
  Approximates the container working set (actively used memory).

- **mem_limit_mb**  
  Container memory limit (MB) reported by Docker (`memory_stats.limit`).  
  If the reported limit is extremely large (treated as unlimited), this value is recorded as `null`.

- **mem_percent**  
  Container memory usage as a percentage of the configured memory limit:
  ```
  mem_percent = 100 × (mem_mb / mem_limit_mb)
  ```
  If no effective memory limit is configured, this value is recorded as `null`.

### Disk I/O Metrics

- **blk_read_mb**  
  Cumulative block device data read by the container, expressed in MB.  
  Obtained from Docker `blkio_stats` when available, with fallbacks to:
  cgroup v1 blkio files, cgroup v2 `io.stat`, host `/proc/<pid>/io`, or exec-based reads inside the container.  
  This is a **cumulative counter**, not a throughput rate.

- **blk_write_mb**  
  Cumulative block device data written by the container, expressed in MB.  
  Same sourcing and semantics as `blk_read_mb`.  
  This is a **cumulative counter**, not a throughput rate.

### Network I/O Metrics

- **net_rx_mb**  
  Cumulative network data received by the container across all interfaces, expressed in MB,  
  obtained from Docker stats `networks.*.rx_bytes`.  
  This is a **cumulative counter**, not a throughput rate.

- **net_tx_mb**  
  Cumulative network data transmitted by the container across all interfaces, expressed in MB,  
  obtained from Docker stats `networks.*.tx_bytes`.  
  This is a **cumulative counter**, not a throughput rate.

## Timestamp Semantics

- **timestamp**  
  Unix epoch timestamp in milliseconds (UTC).

- **timestamp_iso**  
  ISO-8601 formatted timestamp in UTC.

## Notes on Semantics and Accuracy

- CPU and memory metrics are derived from Docker’s native resource accounting and reflect container usage relative to host resources.  
- Disk and network metrics are cumulative byte counters; per-interval throughput (e.g., MB/s) can be computed offline by differentiating consecutive samples.

## Docker Stats – Usage

Check monitoring status:
```shell
curl localhost:6000/monitor/status | jq
```

Start monitoring (stdout only):
```shell
curl -X POST localhost:6000/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{
    "containers": ["alpine1", "alpine2"],
    "interval": 1.0,
    "csv_dir": null,
    "stdout": true
  }' | jq
```

Start monitoring and export results to CSV:
```shell
curl -X POST localhost:6000/monitor/start \
  -H 'Content-Type: application/json' \
  -d '{
    "containers": ["alpine1", "alpine2"],
    "interval": 1.0,
    "csv_dir": "/results/experiments/docker-stats",
    "csv_names": {
      "alpine1": "experimentA-alpine1.csv",
      "alpine2": "experimentA-alpine2.csv"
    },
    "stdout": false
  }' | jq
```

Stop monitoring:
```shell
curl -X POST http://localhost:6000/monitor/stop | jq
```