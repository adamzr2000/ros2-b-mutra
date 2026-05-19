#!/usr/bin/env bash
# observe.sh - snapshot of the running B-MuTRA stack.
#
# Single-shot: prints a human-readable summary on stderr and one JSON object
# on stdout. Read-only, no state on disk, no toolchain beyond curl + jq + docker.
#
# Usage:
#   ./tools/observe.sh                                         # one snapshot
#   watch -n 30 ./tools/observe.sh                             # live monitoring
#   while sleep 30; do ./tools/observe.sh >> snapshots.jsonl; done   # archival
#
# Exit code is 0 even if some probes fail — partial data is better than no data
# when the stack is still spinning up. Missing fields land as JSON `null`.

set -uo pipefail

REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
ADDR_FILE="$REPO/smart-contracts/deployments/besu-AttestationManagerLV.json"

# keccak256 of the three events and the 4-byte selectors of two view fns.
# Regenerate with:
#   docker run --rm --network quorum-dev-quickstart hardhat:latest bash -c "node -e \"
#     const e = require('ethers');
#     console.log(e.id('AttestationStarted(bytes32,uint32)'));
#     console.log(e.id('AttestationCompleted(bytes32,bool)'));
#     console.log(e.id('VerifierRotated(address)'));
#     console.log(e.id('GetVRP()').slice(0,10));
#     console.log(e.id('GetCurrentVerifier()').slice(0,10));
#   \""
TOPIC_STARTED="0x6aa34f034d08ce6962560c4805f8d0ea14020343310408c935a7354c772f2d38"
TOPIC_COMPLETED="0x02d0717bc91617bee67bd08dfacab8f3b5e93100bd242984d1e60ce2758ea475"
TOPIC_ROTATED="0xfcc2be04bc026cce0c24562eefa6383157a6bed7617873597fdd1bbcdeb2b763"
SEL_GET_VRP="0xbb7b8c0c"
SEL_GET_VERIFIER="0x7cb98689"

now_iso() { date -u +%Y-%m-%dT%H:%M:%SZ; }

rpc() {
  # $1 = method, $2 = params (JSON array)
  curl -s --max-time 5 \
    -X POST -H "Content-Type: application/json" \
    --data "{\"jsonrpc\":\"2.0\",\"method\":\"$1\",\"params\":$2,\"id\":1}" \
    http://localhost:21001 2>/dev/null
}

http_get() {
  # $1 = URL — short timeout, silent on failure
  curl -s --max-time 2 "$1" 2>/dev/null
}

# ──────────────────────────────────────────────────────────────────────────
# 1. Containers
# ──────────────────────────────────────────────────────────────────────────
running=$(docker ps --format '{{.Names}}' 2>/dev/null)
n_sidecars=$(echo "$running" | grep -cE '\-sidecar$' || true)
n_robots=$(echo "$running" | grep -cE '^robot[0-9]+$' || true)
n_validators=$(echo "$running" | grep -cE 'validator[0-9]' || true)
secaas_container=$(echo "$running" | grep -cE '^secaas$' || true)

# ──────────────────────────────────────────────────────────────────────────
# 2. Workers (HTTP /status)
# ──────────────────────────────────────────────────────────────────────────
secaas_up=$(http_get http://localhost:8000/ | jq -r 'try .message // "down"')
[[ "$secaas_up" == *running* ]] && secaas_state="running" || secaas_state="$secaas_up"

worker_json="{}"
for i in 1 2 3 4; do
  st=$(http_get "http://localhost:800$i/status" | jq -r 'try .status // "unreachable"')
  worker_json=$(echo "$worker_json" | jq --arg k "robot$i" --arg v "$st" '.[$k]=$v')
done

# ──────────────────────────────────────────────────────────────────────────
# 3. Config (env from a running sidecar + chain reads)
# ──────────────────────────────────────────────────────────────────────────
itq=$(docker exec robot1-sidecar printenv ITERQ_THRESHOLD 2>/dev/null || echo "")
ssp_ms=$(docker exec robot1-sidecar printenv ATTESTATION_INTERVAL_MS 2>/dev/null || echo "")
cpu_limit=$(docker inspect robot1-sidecar --format '{{.HostConfig.NanoCpus}}' 2>/dev/null | awk '{printf "%.2f", $1/1e9}')

ADDR=$(jq -r .address "$ADDR_FILE" 2>/dev/null || echo "")
vrp_raw=$(rpc eth_call "[{\"to\":\"$ADDR\",\"data\":\"$SEL_GET_VRP\"},\"latest\"]" | jq -r 'try .result // ""')
vrp=$([[ -n "$vrp_raw" && "$vrp_raw" != "0x" ]] && printf '%d' "$vrp_raw" 2>/dev/null || echo "")

cv_raw=$(rpc eth_call "[{\"to\":\"$ADDR\",\"data\":\"$SEL_GET_VERIFIER\"},\"latest\"]" | jq -r 'try .result // ""')
current_verifier=""
if [[ -n "$cv_raw" && "$cv_raw" != "0x" ]]; then
  # address is last 40 hex chars
  current_verifier="0x${cv_raw: -40}"
fi

# ──────────────────────────────────────────────────────────────────────────
# 4. Chain state — single eth_getLogs call, group locally
# ──────────────────────────────────────────────────────────────────────────
block_hex=$(rpc eth_blockNumber '[]' | jq -r 'try .result // "0x0"')
block=$((block_hex))

# Besu caps eth_getLogs at 5000 blocks per request. Paginate, accumulating
# each chunk to a tmp file so we don't blow argv with huge JSON strings.
LOG_TMP=$(mktemp /tmp/observe-logs.XXXXXX.jsonl)
trap 'rm -f "$LOG_TMP"' EXIT
get_logs_paginated() {
  local chunk=5000
  local from=0
  while (( from <= block )); do
    local to=$(( from + chunk - 1 ))
    (( to > block )) && to=$block
    rpc eth_getLogs "[{\"fromBlock\":\"$(printf '0x%x' $from)\",\"toBlock\":\"$(printf '0x%x' $to)\",\"address\":\"$ADDR\"}]" \
      | jq -c 'try .result[]? // empty' >> "$LOG_TMP"
    from=$(( to + 1 ))
  done
}
get_logs_paginated

# One jq pass over the events file, emits all counters + the list of
# blockNumbers of pending Started events (their ID is not in any Completed).
# Hashtable lookup (object key) is O(1), the whole pipeline is one read.
metrics=$(jq -s "
  ([.[] | select(.topics[0]==\"$TOPIC_COMPLETED\") | {(.topics[1]): true}] | add // {}) as \$done |
  {
    n_started:       ([.[] | select(.topics[0]==\"$TOPIC_STARTED\")] | length),
    n_completed:     ([.[] | select(.topics[0]==\"$TOPIC_COMPLETED\")] | length),
    n_rotated:       ([.[] | select(.topics[0]==\"$TOPIC_ROTATED\")] | length),
    n_success:       ([.[] | select(.topics[0]==\"$TOPIC_COMPLETED\") | select(.data[-1:]==\"1\")] | length),
    n_failure:       ([.[] | select(.topics[0]==\"$TOPIC_COMPLETED\") | select(.data[-1:]==\"0\")] | length),
    pending_blocks:  [.[] | select(.topics[0]==\"$TOPIC_STARTED\") | select(\$done[.data[0:66]] | not) | .blockNumber]
  }
" "$LOG_TMP")

n_started=$(echo   "$metrics" | jq -r '.n_started')
n_completed=$(echo "$metrics" | jq -r '.n_completed')
n_rotated=$(echo   "$metrics" | jq -r '.n_rotated')
n_success=$(echo   "$metrics" | jq -r '.n_success')
n_failure=$(echo   "$metrics" | jq -r '.n_failure')
in_flight=$((n_started - n_completed))

# Age (in seconds) of the oldest unclosed attestation — the wall-clock
# latency the most-recent attestation would face if the verifier is FIFO.
# Hex strings sort lexicographically which is wrong when lengths differ,
# so we let bash do the minimum in decimal.
oldest_age_s=""
oldest_pending_block=""
pending_blocks=$(echo "$metrics" | jq -r '.pending_blocks[]?')
if [[ -n "$pending_blocks" ]]; then
  for hex in $pending_blocks; do
    dec=$((hex))
    if [[ -z "$oldest_pending_block" || $dec -lt $oldest_pending_block ]]; then
      oldest_pending_block=$dec
    fi
  done
  if [[ -n "$oldest_pending_block" ]]; then
    old_ts_hex=$(rpc eth_getBlockByNumber "[\"$(printf '0x%x' "$oldest_pending_block")\", false]" \
      | jq -r 'try .result.timestamp // ""')
    latest_ts_hex=$(rpc eth_getBlockByNumber "[\"latest\", false]" \
      | jq -r 'try .result.timestamp // ""')
    if [[ -n "$old_ts_hex" && -n "$latest_ts_hex" ]]; then
      oldest_age_s=$(( latest_ts_hex - old_ts_hex ))
    fi
  fi
fi

# ──────────────────────────────────────────────────────────────────────────
# 5. Sidecar-side counters (from logs, cumulative since container boot)
# ──────────────────────────────────────────────────────────────────────────
log_success=0; log_failure=0
for c in secaas robot1-sidecar robot2-sidecar robot3-sidecar robot4-sidecar; do
  s=$(docker logs "$c" 2>&1 | grep -c "SUCCESS" || true)
  f=$(docker logs "$c" 2>&1 | grep -c "FAILURE" || true)
  log_success=$((log_success + s))
  log_failure=$((log_failure + f))
done

# ──────────────────────────────────────────────────────────────────────────
# 6. Stdout: one JSON line. Stderr: human summary.
# ──────────────────────────────────────────────────────────────────────────
ts=$(now_iso)

snapshot=$(jq -n \
  --arg ts "$ts" \
  --argjson sidecars "$n_sidecars" --argjson robots "$n_robots" \
  --argjson validators "$n_validators" --argjson secaas_container "$secaas_container" \
  --arg secaas_state "$secaas_state" \
  --argjson workers "$worker_json" \
  --arg itq "${itq:-null}" --arg ssp_ms "${ssp_ms:-null}" --arg cpu_limit "${cpu_limit:-null}" \
  --arg vrp "${vrp:-null}" --arg cv "${current_verifier:-null}" \
  --argjson block "$block" \
  --argjson n_started "$n_started" --argjson n_completed "$n_completed" \
  --argjson n_success "$n_success" --argjson n_failure "$n_failure" \
  --argjson n_rotated "$n_rotated" --argjson in_flight "$in_flight" \
  --arg oldest_age_s "${oldest_age_s:-null}" \
  --arg oldest_pending_block "${oldest_pending_block:-null}" \
  --argjson log_success "$log_success" --argjson log_failure "$log_failure" \
  '{
    ts: $ts,
    containers: { sidecars: $sidecars, robots: $robots, secaas: ($secaas_container==1), validators: $validators },
    workers: ({secaas: $secaas_state} + $workers),
    config: {
      iterq:   ($itq       | if . == "null" or . == "" then null else tonumber end),
      ssp_ms:  ($ssp_ms    | if . == "null" or . == "" then null else tonumber end),
      cpu_limit: ($cpu_limit | if . == "null" or . == "" then null else tonumber end),
      vrp_onchain: ($vrp   | if . == "null" or . == "" then null else tonumber end),
      current_verifier: ($cv | if . == "null" or . == "" then null else . end)
    },
    chain: {
      block: $block,
      events_started: $n_started, events_completed: $n_completed,
      completed_success: $n_success, completed_failure: $n_failure,
      in_flight: $in_flight, verifier_rotations: $n_rotated,
      oldest_pending_block: ($oldest_pending_block | if . == "null" or . == "" then null else tonumber end),
      oldest_pending_age_s: ($oldest_age_s | if . == "null" or . == "" then null else tonumber end)
    },
    logs: { success: $log_success, failure: $log_failure }
  }')

echo "$snapshot" | jq -c .  # stdout: one JSON line

# Human-readable summary on stderr
{
  echo
  echo "=== B-MuTRA snapshot @ $ts ==="
  printf '  containers: %d sidecars, %d robots, %d validators, secaas=%s\n' \
    "$n_sidecars" "$n_robots" "$n_validators" "$( ((secaas_container==1)) && echo up || echo DOWN )"
  printf '  workers:    secaas=%s   ' "$secaas_state"
  echo "$worker_json" | jq -r 'to_entries[] | "\(.key)=\(.value)"' | tr '\n' ' '
  echo
  printf '  config:     K=%s   SSP=%sms   CPU=%s   VRP=%s   verifier=%s\n' \
    "${itq:-?}" "${ssp_ms:-?}" "${cpu_limit:-?}" "${vrp:-?}" "${current_verifier:0:14}…"
  printf '  chain:      block=%d   started=%d   completed=%d   success=%d   failure=%d   in_flight=%d   rotations=%d\n' \
    "$block" "$n_started" "$n_completed" "$n_success" "$n_failure" "$in_flight" "$n_rotated"
  if [[ -n "$oldest_age_s" ]]; then
    h=$(( oldest_age_s / 3600 )); m=$(( (oldest_age_s % 3600) / 60 )); s=$(( oldest_age_s % 60 ))
    printf '  backlog:    oldest pending = block %d, %dh%02dm%02ds old\n' \
      "$oldest_pending_block" "$h" "$m" "$s"
  fi
  printf '  logs:       success=%d   failure=%d\n' "$log_success" "$log_failure"
  echo
} >&2
