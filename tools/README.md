# tools/

Operational helpers that don't need to be on the critical path of the
stack itself. Read-only or self-contained.

- `observe.sh` — single-shot snapshot of stack state and chain counters
- `tamper.sh`  — inject in-memory binary tampering for failure-detection tests

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
    "iterq": 1, "ssp_ms": 20000, "cpu_limit": 0.4,
    "current_verifier": "0x0ba0690b..."
  },
  "chain": {
    "block": 7669,
    "events_started": 100, "events_completed": 96,
    "in_flight": 4,
    "oldest_pending_block": 7012,
    "oldest_pending_age_s": 42
  },
  "logs": { "success": 192, "failure": 0 }
}
```

## Reading the numbers

- **`config.iterq`** — sanity check that what you asked for on the CLI
  actually made it into the running stack.
- **`chain.events_started` vs `events_completed`** — should track. The
  diff (`in_flight`) is the backlog of attestations submitted but not
  yet closed. If it grows monotonically across snapshots, the
  verifier or oracle is stuck.
- **`logs.success` / `logs.failure`** — the ground truth for tamper
  detection. Note: each attestation produces one log entry in the
  prover container AND one in the verifier container, so the raw counts
  are ~2× the actual number of attestations. `failure > 0` means at
  least one measurement didn't match the reference (tamper detected, or
  bootstrap was skipped).
- **`chain.oldest_pending_age_s`** — direct measurement of the longest
  wait an attestation currently has, in seconds. Computed as
  `block.timestamp(latest) - block.timestamp(oldest_pending_started)`.
  If this number grows monotonically across snapshots, the verifier
  pipeline is not draining fast enough and your end-to-end latency
  will keep climbing. If it stays flat or oscillates, the system has
  reached a steady close-rate that matches its start-rate.
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

## tamper.sh

In-memory tampering of a binary's `.text` segment. Writes a single
NOP byte at a configurable offset via `/proc/<pid>/mem` from inside
the sidecar container, which has CAP_SYS_PTRACE and shares the PID
namespace with its robot. The change is in-memory only; restart the
affected container to undo.

The script is pure injection: it writes the byte and exits. To see
whether the framework detects the corruption as a FAILURE attestation,
watch `./tools/observe.sh` (chain.completed_failure) or container
logs separately.

```bash
# tamper robot_state_publisher in robot1 (default target)
./tools/tamper.sh robot1

# tamper the sidecar itself (self-integrity check, needs SELF_INTEGRITY_ENABLED)
./tools/tamper.sh robot1 --target sidecar

# tamper dummy_publisher in dummy-mode robot
./tools/tamper.sh robot2 --target dummy

# custom offset
./tools/tamper.sh robot1 --target sidecar --offset 0x80000
```

To observe the verdict, run an observe.sh snapshot before and after:

```bash
pre=$(./tools/observe.sh 2>/dev/null | jq -r .chain.completed_failure)
./tools/tamper.sh robot1
sleep 60
post=$(./tools/observe.sh 2>/dev/null | jq -r .chain.completed_failure)
echo "new failures: $(( post - pre ))"
```

