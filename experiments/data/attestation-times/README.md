# Attestation Duration Data

## Overview

`summarize_attestation_durations.py` reads per-attestation timing JSON files produced by the sidecar and SECaaS containers, aggregates them into per-run means, and writes clean CSVs consumed by the plot scripts.

---

## Input layout

```
results/
  N{n}/
    startup/
      {participant}-run{N}.json
    continuous/
      {rr|lv}/
        {participant}-SSP{ssp}ms-ITERQ{k}-cpu{cpu}-run{N}.json
```

- `N{n}` — fleet size (e.g. `N8`, `N32`)
- `{participant}` — `robot1`…`robotN` or `secaas`
- `SSP{ssp}ms` — sidecar sleep period in ms (e.g. `SSP20000ms` = 20 s)
- `ITERQ{k}` — rolling-hash depth (e.g. `ITERQ1` = single measurement)
- `cpu{cpu}` — CPU fraction cap (e.g. `cpu0p4` = 0.4, `cpuNC` = uncapped)
- `run{N}` — independent run index (1-based)

Each JSON contains three top-level arrays (`prover`, `verifier`, `oracle`), one object per attestation cycle completed during that run.

---

## What is measured

Each timing object is parsed for the following metrics per role:

| Role | Metric | Description |
|------|--------|-------------|
| **prover** | `total_lifecycle` | Full prover wall time (start → result received) |
| | `e2e_blockchain` | From `SendEvidence` TX submission to result received |
| | `evidence_call` | `SendEvidence` RPC call duration |
| | `evidence_tx_confirm` | Time until TX is confirmed on-chain |
| **verifier** | `total_lifecycle` | Full verifier wall time |
| | `signatures_fetch` | `GetAttestationSignatures` blockchain call |
| | `verify_compute` | Local hash comparison |
| | `result_call` | `CloseAttestationProcess` TX call |
| | `result_tx_confirm` | Time until TX is confirmed |
| | `reaction_time` | Lag from event received to processing start |
| **oracle** (SECaaS) | `total_lifecycle` | Full oracle wall time |
| | `prover_addr_fetch` | `GetProverAddress` blockchain call |
| | `db_lookup` | Reference measurement fetch from PostgreSQL |
| | `ref_signature_call` | `SendRefSignature` TX call |
| | `ref_signature_tx_confirm` | Time until TX is confirmed |
| | `reaction_time` | Lag from event received to processing start |

Values are extracted in priority order: native nanoseconds (`_ns`) → pre-calculated milliseconds (`_ms`) → wall-clock timestamp subtraction.

---

## Aggregation pipeline

```
raw cycles (one object per attestation)
        │
        ▼
  mean across all cycles within a (participant, run)
        │   gives equal weight to each independent run
        ▼
  durations_per_run_*.csv          ← primary output, used by all plots
```

The key design decision is averaging **within a run first**: a run with 100 completed cycles does not outweigh one with 30. The per-run mean is the atomic unit; plots then compute mean ± std across the 5 independent runs.

---

## Output files

All files are written to `_summary/`.

### `durations_per_run_startup.csv`

One row per `(n_robots, participant, run, role, metric)`.

| Column | Description |
|--------|-------------|
| `n_robots` | Fleet size |
| `participant` | e.g. `Robot1`, `SECaaS` |
| `run` | Run index |
| `role` | `prover`, `verifier`, or `oracle` |
| `metric` | Metric name (see table above) |
| `run_median_s` | Median duration across cycles in this run (seconds) |
| `participant_group` | `Robot` or `SECaaS` (for grouped aggregation in plots) |

### `durations_per_run_rr.csv` / `durations_per_run_lv.csv`

Same as startup, plus three extra columns describing the experimental condition:

| Column | Description |
|--------|-------------|
| `ssp_ms` | Sidecar sleep period (ms) |
| `iterq` | IterQ rolling-hash depth |
| `cpu_limit` | CPU fraction cap (`None` = uncapped) |

A file is written only if data for that contract exists in `results/`.

---

## Configuration flags

| Flag | Default | Effect |
|------|---------|--------|
| `EXPORT_RAW` | `False` | Write `durations_raw_*.csv` — one row per individual cycle, large |
| `EXPORT_SUMMARY` | `False` | Write `durations_summary_*.csv` — across-run mean/std/min/max/p25/p75; not used by plots |

The primary output (`durations_per_run_*.csv`) is always written.
