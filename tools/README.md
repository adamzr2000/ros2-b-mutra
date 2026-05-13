# tools/

Single-shot snapshot of the running B-MuTRA stack.

## observe.sh

Read-only. ~1-2s per call. Writes a structured JSON object to stdout
and a human-readable summary to stderr.

```bash
# One snapshot, human-readable on the terminal
./tools/observe.sh

# Live monitoring (refresh every 30s)
watch -n 30 ./tools/observe.sh

# Archival data collection (JSON lines, append-only)
while sleep 30; do ./tools/observe.sh; done >> snapshots.jsonl 2>/dev/null

# Just the human side (skip the JSON noise)
./tools/observe.sh > /dev/null
```

## What's in a snapshot

```json
{
  "ts": "2026-05-13T14:25:32Z",
  "containers": { "sidecars": 4, "robots": 4, "secaas": true, "validators": 4 },
  "workers": { "secaas": "running", "robot1": "running", ... },
  "config": {
    "iterq": 500, "ssp_ms": 0, "cpu_limit": 0.4,
    "vrp_onchain": 100, "current_verifier": "0x0ba0690b..."
  },
  "chain": {
    "block": 7669,
    "events_started": 1100, "events_completed": 366,
    "completed_success": 366, "completed_failure": 0,
    "in_flight": 734, "verifier_rotations": 3
  },
  "logs": { "success": 629, "failure": 0 }
}
```

## Reading the numbers

- **`config.iterq` / `config.vrp_onchain`** — sanity check that what you
  asked for on the CLI actually made it into the running stack.
- **`chain.events_started` vs `events_completed`** — should track. The
  diff (`in_flight`) is the backlog of attestations submitted but not
  yet closed. If it grows monotonically across snapshots, the
  verifier or oracle is stuck.
- **`chain.completed_success` / `completed_failure`** — the ground
  truth. `completed_failure > 0` means refs and measurements don't
  match (you forgot to bootstrap, or you rebuilt a sidecar without
  re-bootstrapping).
- **`chain.verifier_rotations`** — should equal
  `completed_success ÷ vrp_onchain` (modulo failure stores). With
  `vrp_onchain=100`, you get 1 rotation per 100 successes.
- **`logs.success` / `logs.failure`** — secaas + sidecar log
  grep-counts since container boot. Useful as a cross-check against
  the on-chain events.

## Caveats

- Pulls all events from block 0 every call, paginated in 5000-block
  chunks. Quick on a small chain (under 2s for ~10k blocks); will
  grow O(blocks) as the chain ages. If you ever care about deltas
  rather than totals, add a `--from-block` flag.
- `eth_call` to `GetVRP()` and `GetCurrentVerifier()` uses 4-byte
  function selectors hardcoded at the top of the script. If the
  contract changes those signatures, regenerate them (the comment
  block at the top of the script shows how).
- `logs.success` / `logs.failure` count substring matches of "SUCCESS"
  and "FAILURE" in container logs — they may double-count if either
  word appears for non-attestation reasons. Treat them as
  approximations; the on-chain `completed_*` numbers are the source
  of truth.
