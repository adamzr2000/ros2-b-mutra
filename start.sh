#!/bin/bash
set -euo pipefail
IFS=$'\n\t'

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ENV_FILE="$SCRIPT_DIR/.env"
SECAAS_URL="http://localhost:8000"
PROJECT_NAME="ros2-b-mutra"   # one Compose project for every stack

# Remote hosts (--remote). Defaults: the 5TONIC bench. Override via the
# environment; stop.sh reads the same variables.
REMOTE1_HOST="${REMOTE1_HOST:-10.5.1.20}"
REMOTE1_USER="${REMOTE1_USER:-nextnet}"
REMOTE1_LIMIT=64    # robots 1..64 on remote1
REMOTE2_HOST="${REMOTE2_HOST:-}"
REMOTE2_USER="${REMOTE2_USER:-nextnet}"
REMOTE2_LIMIT=128   # robots 65..128 on remote2
REMOTE_DIR="~/ros2-b-mutra"

# This host's IP as seen from the remote hosts: their sidecars reach Besu here.
LOCAL_IP="${LOCAL_IP:-10.5.99.99}"

export COMPOSE_IGNORE_ORPHANS=1   # several compose files share one project

run_remote() {
  local host="$1"
  local user="$2"
  shift 2
  ssh -o BatchMode=yes -o ConnectTimeout=10 "$user@$host" "$@"
}

remote_compose_up() {
  local host="$1"
  local user="$2"
  local compose_file="$3"
  local retries=3
  local attempt
  for ((attempt = 1; attempt <= retries; attempt++)); do
    if run_remote "$host" "$user" \
        "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME --project-directory $REMOTE_DIR -f $compose_file up -d"; then
      return 0
    fi
    if (( attempt < retries )); then
      echo "⚠️  Remote compose failed (attempt $attempt/$retries), retrying in 5s..."
      sleep 5
    fi
  done
  return 1
}

usage() {
  cat <<EOF
Usage: $(basename "$0") [options]

Deployment
  --robots N         Robots to deploy (default: 4). Gazebo: 1..16; --dummy: 1..$REMOTE2_LIMIT.
  --dummy            Robots run a dummy ROS 2 publisher (attested) instead of the
                     Gazebo simulation: no simulator, for large N.
  --remote           Robots and sidecars on the remote hosts (remote1: 1..$REMOTE1_LIMIT,
                     remote2: above); this host keeps Besu, SECaaS and monitoring.
                     Default: everything on this host.
  --contract C       lv | v2 | v3 (default: v3)
                       lv  the last verified robot verifies the next one
                       v2  lv + references anchored on chain by admin keys
                       v3  v2 + a jury of verifiers
  --jury N           v3: jurors per cycle (default: 2)
  --quorum T         v3: matching votes that close a cycle, 1..N (default: N; any
                     disagreement then goes to the SECaaS, whose verdict is final)

Attestation
  --auto             Start the attestation workers once the stack is up
  --ssp MS           Delay between measurements in ms (default: 10000; 0 = back to back)
  --ssp-mode M       fixed: wait exactly the SSP; exp: exponential draw of mean SSP
                     (default: fixed)
  --iterq K          Measurements folded into one on-chain attestation, 1..10000 (default: 1)
  --cpu-limit X      Sidecar CPU cap in cores (default: 0.4)
  --no-cpu-limit     No sidecar CPU cap (results tagged cpuNC)
  --rounds N         Rounds per robot, then its prover stops (default: 0 = until stopped)
  --no-wait-result   Provers do not wait for the verdict
  --wait-tx          Wait for each transaction to be mined
  --no-bootstrap     Keep the placeholder references (attestations then FAIL). By
                     default robot1's real hashes become every robot's reference.

Attested process (default: robot_state_publisher; dummy_publisher with --dummy)
  --attest-gzserver       gzserver (local, --robots 4)
  --attest-mission-agent  mission_agent (requires --nav2)

Simulation (Gazebo only)
  --world NAME       empty | turtlebot3_world | swarm_arena | house | dqn1..dqn4 (default: empty)
  --gpu              NVIDIA GPU for gazebo-server (needs the NVIDIA Container Toolkit)
  --gui              Gazebo client in the browser at http://localhost:6080 (alias: --vnc)
  --nav2             Nav2 on every robot

Experiments (local Gazebo, N≤4)
  --mission          Waypoint mission orchestrator (requires --nav2)
  --formation        Formation experiment: formation_agent on every robot, attested,
                     plus its orchestrator. Excludes --nav2, --mission, --attest-*.
  --formation-scale D  Half-diagonal of the square formation in m (default: 2.0)
  --mitigation on|off  On attestation FAILURE, isolate the robot (on) or only log it
                     (default: on)
  --run-tag TAG      Tag of the orchestrator output files (default: run)

Monitoring
  --dashboards       Block explorer, Prometheus, Grafana and Loki (default: off).
                     The resource collector used by the experiments always runs.

  -h, --help         Show this help

EOF
}

# Default values
N_ROBOTS_VAL="4"
DUMMY_VAL="FALSE"
AUTO_START_VAL="FALSE"
WAIT_TX_VAL="FALSE"
ROUNDS_VAL="0"
DEPLOY_MODE="local"
SSP_VAL="10000"
SSP_MODE_VAL="fixed"
CPU_LIMIT_VAL="0.4"
NO_CPU_LIMIT_VAL="FALSE"
CPU_LIMIT_EXPLICIT="FALSE"
VARIANT_VAL="v3"
JURY_VAL="2"
QUORUM_VAL=""  # empty: unanimous, the jury size
ITERQ_VAL="1"
BOOTSTRAP_REFS="TRUE"
WAIT_RESULT_VAL="TRUE"
ATTEST_GZSERVER_VAL="FALSE"
GPU_VAL="FALSE"
GUI_VAL="FALSE"
DASHBOARDS_VAL="FALSE"
WORLD_VAL="empty"
NAV2_VAL="FALSE"

ATTEST_MISSION_AGENT_VAL="FALSE"
MISSION_VAL="FALSE"
MITIGATION_VAL="true"
RUN_TAG_VAL="run"
FORMATION_VAL="FALSE"
FORMATION_SCALE_VAL="2.0"

# Bytes each sidecar hashes, from the start of the process's first executable
# mapping (+ offset). Re-derive after rebuilding the binary:
#   readelf -lW <bin> | awk '/LOAD/ && /R E/'   (memsz of the R E segment)
GZSERVER_TEXT_SECTION_SIZE="85412"           # same gzserver binary in gazebo-server
GZSERVER_TEXT_SECTION_OFFSET="35552"
MISSION_AGENT_TEXT_SECTION_SIZE="796065"     # covers tick(), the tampered function
MISSION_AGENT_TEXT_SECTION_OFFSET="0"
FORMATION_AGENT_TEXT_SECTION_SIZE="1019085"  # covers steer_cmd(), the tampered function
FORMATION_AGENT_TEXT_SECTION_OFFSET="0"

# Parse args
while [[ $# -gt 0 ]]; do
  case "$1" in
    --robots)
      N_ROBOTS_VAL="$2"
      if ! [[ "$N_ROBOTS_VAL" =~ ^[1-9][0-9]*$ ]] || (( N_ROBOTS_VAL < 1 || N_ROBOTS_VAL > REMOTE2_LIMIT )); then
        echo "❌ --robots must be an integer between 1 and $REMOTE2_LIMIT (got '$N_ROBOTS_VAL')"
        exit 1
      fi
      shift 2 ;;
    --remote)   DEPLOY_MODE="remote"; shift ;;
    --auto)     AUTO_START_VAL="TRUE"; shift ;;
    --wait-tx)  WAIT_TX_VAL="TRUE"; shift ;;
    --rounds)
      ROUNDS_VAL="$2"
      if ! [[ "$ROUNDS_VAL" =~ ^(0|[1-9][0-9]*)$ ]]; then
        echo "❌ --rounds must be a non-negative integer (got '$ROUNDS_VAL')"
        exit 1
      fi
      shift 2 ;;
    --ssp)
      SSP_VAL="$2"
      if ! [[ "$SSP_VAL" =~ ^(0|[1-9][0-9]*)$ ]]; then
        echo "❌ --ssp must be a non-negative integer in milliseconds (got '$SSP_VAL')"
        exit 1
      fi
      shift 2 ;;
    --ssp-mode)
      SSP_MODE_VAL="$2"
      if [[ "$SSP_MODE_VAL" != "fixed" && "$SSP_MODE_VAL" != "exp" ]]; then
        echo "❌ --ssp-mode must be fixed or exp (got '$SSP_MODE_VAL')"
        exit 1
      fi
      shift 2 ;;
    --cpu-limit)
      CPU_LIMIT_VAL="$2"
      CPU_LIMIT_EXPLICIT="TRUE"
      if ! [[ "$CPU_LIMIT_VAL" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "❌ --cpu-limit must be a positive number (got '$CPU_LIMIT_VAL')"
        exit 1
      fi
      shift 2 ;;
    --no-cpu-limit) NO_CPU_LIMIT_VAL="TRUE"; shift ;;
    --contract)
      VARIANT_VAL="$2"
      if [[ "$VARIANT_VAL" != "lv" && "$VARIANT_VAL" != "v2" && "$VARIANT_VAL" != "v3" ]]; then
        echo "❌ --contract must be 'lv', 'v2' or 'v3' (got '$VARIANT_VAL')"
        exit 1
      fi
      shift 2 ;;
    --jury|--quorum)
      [[ "$2" =~ ^[1-9][0-9]*$ ]] || { echo "❌ $1 must be a positive integer (got '$2')"; exit 1; }
      if [[ "$1" == "--jury" ]]; then JURY_VAL="$2"; else QUORUM_VAL="$2"; fi
      shift 2 ;;
    --iterq)
      ITERQ_VAL="$2"
      if ! [[ "$ITERQ_VAL" =~ ^[1-9][0-9]*$ ]] || (( ITERQ_VAL > 10000 )); then
        echo "❌ --iterq must be an integer in 1..10000 (got '$ITERQ_VAL')"
        exit 1
      fi
      shift 2 ;;
    --no-bootstrap)  BOOTSTRAP_REFS="FALSE"; shift ;;
    --attest-gzserver) ATTEST_GZSERVER_VAL="TRUE"; shift ;;
    --attest-mission-agent) ATTEST_MISSION_AGENT_VAL="TRUE"; shift ;;
    --mission)       MISSION_VAL="TRUE"; shift ;;
    --formation)     FORMATION_VAL="TRUE"; shift ;;
    --formation-scale)
      FORMATION_SCALE_VAL="$2"
      if ! [[ "$FORMATION_SCALE_VAL" =~ ^[0-9]+(\.[0-9]+)?$ ]]; then
        echo "❌ --formation-scale must be a positive number (got '$FORMATION_SCALE_VAL')"; exit 1
      fi
      shift 2 ;;
    --mitigation)
      MITIGATION_VAL="$2"
      if [[ "$MITIGATION_VAL" != "on" && "$MITIGATION_VAL" != "off" ]]; then
        echo "❌ --mitigation must be 'on' or 'off' (got '$MITIGATION_VAL')"; exit 1
      fi
      [[ "$MITIGATION_VAL" == "on" ]] && MITIGATION_VAL="true" || MITIGATION_VAL="false"
      shift 2 ;;
    --run-tag)
      if [[ -z "${2-}" ]]; then echo "❌ --run-tag requires a value"; exit 1; fi
      RUN_TAG_VAL="$2"; shift 2 ;;
    --no-wait-result) WAIT_RESULT_VAL="FALSE"; shift ;;
    --gpu)           GPU_VAL="TRUE"; shift ;;
    --gui|--vnc)     GUI_VAL="TRUE"; shift ;;
    --dashboards)    DASHBOARDS_VAL="TRUE"; shift ;;
    --world)
      WORLD_VAL="$2"
      case "$WORLD_VAL" in
        empty|turtlebot3_world|swarm_arena|house|dqn1|dqn2|dqn3|dqn4) ;;
        *) echo "❌ --world must be one of: empty, turtlebot3_world, swarm_arena, house, dqn1, dqn2, dqn3, dqn4 (got '$WORLD_VAL')"; exit 1 ;;
      esac
      shift 2 ;;
    --nav2)          NAV2_VAL="TRUE"; shift ;;
    --dummy)         DUMMY_VAL="TRUE"; shift ;;
    -h|--help)  usage; exit 0 ;;
    *) echo "❌ Unknown arg: $1"; echo; usage; exit 1 ;;
  esac
done

if [[ "$NO_CPU_LIMIT_VAL" == "TRUE" && "$CPU_LIMIT_EXPLICIT" == "TRUE" ]]; then
  echo "❌ --no-cpu-limit and --cpu-limit are mutually exclusive"
  exit 1
fi

if [[ "$DUMMY_VAL" == "TRUE" ]]; then
  if [[ "$WORLD_VAL" != "empty" ]]; then
    echo "❌ --world needs Gazebo: not valid with --dummy"; exit 1
  fi
  for opt in GPU GUI NAV2 MISSION FORMATION ATTEST_GZSERVER ATTEST_MISSION_AGENT; do
    var="${opt}_VAL"
    if [[ "${!var}" == "TRUE" ]]; then
      echo "❌ --$(tr 'A-Z_' 'a-z-' <<< "$opt") needs Gazebo: not valid with --dummy"; exit 1
    fi
  done
elif (( N_ROBOTS_VAL > 16 )); then
  echo "❌ Gazebo runs at most 16 robots: add --dummy for more"
  exit 1
fi

if [[ "$ATTEST_GZSERVER_VAL" == "TRUE" ]] && { [[ "$DEPLOY_MODE" != "local" ]] || [[ "$N_ROBOTS_VAL" != "4" ]]; }; then
  echo "❌ --attest-gzserver is only valid with --robots 4 in local mode"
  exit 1
fi

if [[ "$ATTEST_GZSERVER_VAL" == "TRUE" && "$ATTEST_MISSION_AGENT_VAL" == "TRUE" ]]; then
  echo "❌ --attest-gzserver and --attest-mission-agent are mutually exclusive"
  exit 1
fi

# mission_agent only runs with Nav2.
if [[ "$ATTEST_MISSION_AGENT_VAL" == "TRUE" && "$NAV2_VAL" != "TRUE" ]]; then
  echo "❌ --attest-mission-agent requires --nav2 (the mission_agent only runs with Nav2)"
  exit 1
fi

if [[ "$MISSION_VAL" == "TRUE" ]] && { [[ "$NAV2_VAL" != "TRUE" ]] || [[ "$DEPLOY_MODE" != "local" ]] || (( N_ROBOTS_VAL > 4 )); }; then
  echo "❌ --mission requires --nav2 in local Gazebo mode (--robots 1..4)"
  exit 1
fi

if [[ "$FORMATION_VAL" == "TRUE" ]]; then
  if [[ "$DEPLOY_MODE" != "local" ]] || (( N_ROBOTS_VAL > 4 )); then
    echo "❌ --formation requires local Gazebo mode (--robots 1..4)"; exit 1
  fi
  if [[ "$NAV2_VAL" == "TRUE" || "$MISSION_VAL" == "TRUE" ]]; then
    echo "❌ --formation is mutually exclusive with --nav2 / --mission"; exit 1
  fi
  if [[ "$ATTEST_GZSERVER_VAL" == "TRUE" || "$ATTEST_MISSION_AGENT_VAL" == "TRUE" ]]; then
    echo "❌ --formation is mutually exclusive with --attest-gzserver / --attest-mission-agent"; exit 1
  fi
fi

case "$VARIANT_VAL" in
  lv) CONTRACT_VAL="AttestationManagerLV" ;;
  v2) CONTRACT_VAL="AttestationManagerV2" ;;
  v3) CONTRACT_VAL="AttestationManagerV3" ;;
esac

# V2/V3: references are set on chain by admins, never by the SECaaS (validator1).
# Admins: validators 2-4, 2 approvals out of 3.
ADMIN_NODES_DIR="$SCRIPT_DIR/blockchain/quorum-test-network/config/nodes"
if [[ "$CONTRACT_VAL" == AttestationManagerV[23] ]]; then
  ADMINS=""
  for v in 2 3 4; do
    admin_addr="$(jq -er '"0x" + (.address | ltrimstr("0x"))' "$ADMIN_NODES_DIR/validator$v/accountKeystore")" \
      || { echo "❌ cannot read the admin address of validator$v"; exit 1; }
    ADMINS="${ADMINS:+$ADMINS,}$admin_addr"
  done
  export ADMINS
  export ADMIN_THRESHOLD=2
fi
# V3: QUORUM_VAL matching votes out of JURY_VAL close a cycle. A split no quorum
# can resolve, or 5 blocks (10 s) without a close, sends it to the SECaaS.
# Evidence is bound to a block at most FRESHNESS_BLOCKS old when it is sent:
# twice the IterQ batch ((K-1) SSPs), plus 15 blocks for the round to be sent.
if [[ "$CONTRACT_VAL" == "AttestationManagerV3" ]]; then
  export JURY_SIZE="$JURY_VAL" JURY_THRESHOLD="${QUORUM_VAL:-$JURY_VAL}" JURY_TIMEOUT_BLOCKS=5
  block_s="$(jq -er '.config.qbft.blockperiodseconds' "$SCRIPT_DIR/blockchain/quorum-test-network/config/besu/genesis.json")" \
    || { echo "❌ cannot read the block period from genesis.json"; exit 1; }
  FRESHNESS_BLOCKS=$(( (2 * (ITERQ_VAL - 1) * SSP_VAL + block_s * 1000 - 1) / (block_s * 1000) + 15 ))
  if (( FRESHNESS_BLOCKS > 255 )); then
    echo "❌ An IterQ batch of $ITERQ_VAL x ${SSP_VAL}ms needs a freshness window of $FRESHNESS_BLOCKS blocks (max 255): lower --iterq or --ssp"
    exit 1
  fi
  export FRESHNESS_BLOCKS
fi

# V2/V3: measurements and references are salted per agent, so the ledger does
# not reveal which binary a robot runs.
SALT_MEASUREMENT_VAL="FALSE"
BOOTSTRAP_FLAGS=()
if [[ "$CONTRACT_VAL" == AttestationManagerV[23] ]]; then
  SALT_MEASUREMENT_VAL="TRUE"
  BOOTSTRAP_FLAGS=(--salted)
fi

if [[ "$DEPLOY_MODE" == "remote" ]] && (( N_ROBOTS_VAL > REMOTE1_LIMIT )) && [[ -z "$REMOTE2_HOST" ]]; then
  echo "❌ N=$N_ROBOTS_VAL > REMOTE1_LIMIT=$REMOTE1_LIMIT but REMOTE2_HOST is not configured in start.sh."
  exit 1
fi

# ── .env ──────────────────────────────────────────────────────────────────────
upsert_env() {
  local key="$1"
  local val="$2"
  if grep -qE "^[[:space:]]*$key=" "$ENV_FILE"; then
    sed -i "s|^[[:space:]]*$key=.*|$key=$val|g" "$ENV_FILE"
  else
    echo "$key=$val" >> "$ENV_FILE"
  fi
}

if [[ ! -f "$ENV_FILE" ]]; then
  echo "Creating missing .env at $ENV_FILE"
  touch "$ENV_FILE"
fi

# With --auto, containers still boot idle while the bootstrap replaces the
# placeholder references; the workers are started over HTTP afterwards.
WANT_AUTO_START="$AUTO_START_VAL"
EFFECTIVE_AUTO_START="$AUTO_START_VAL"
if [[ "$BOOTSTRAP_REFS" == "TRUE" && "$AUTO_START_VAL" == "TRUE" ]]; then
  EFFECTIVE_AUTO_START="FALSE"   # hold the workers until bootstrap is done
fi

upsert_env "AUTO_START"                "$EFFECTIVE_AUTO_START"
upsert_env "WAIT_FOR_TX_CONFIRMATIONS" "$WAIT_TX_VAL"
upsert_env "WAIT_FOR_VERIFICATION_RESULT"  "$WAIT_RESULT_VAL"
upsert_env "ROUNDS"                    "$ROUNDS_VAL"
upsert_env "N_ROBOTS"                  "$N_ROBOTS_VAL"
upsert_env "CONTRACT"                  "$CONTRACT_VAL"
upsert_env "COMPOSE_PROJECT_NAME"      "$PROJECT_NAME"
upsert_env "DEPLOY_MODE"               "$DEPLOY_MODE"
upsert_env "REMOTE1_HOST"              "$REMOTE1_HOST"
upsert_env "REMOTE2_HOST"              "$REMOTE2_HOST"
upsert_env "ATTESTATION_INTERVAL_MS"  "$SSP_VAL"
upsert_env "ATTESTATION_INTERVAL_MODE" "$SSP_MODE_VAL"
if [[ "$NO_CPU_LIMIT_VAL" == "TRUE" ]]; then
  upsert_env "CPU_LIMIT" "none"
else
  upsert_env "CPU_LIMIT" "$CPU_LIMIT_VAL"
fi
upsert_env "ITERQ_THRESHOLD"           "$ITERQ_VAL"
upsert_env "SALT_MEASUREMENT"          "$SALT_MEASUREMENT_VAL"
upsert_env "GAZEBO_WORLD"              "$WORLD_VAL"
upsert_env "ENABLE_NAV2"              "$NAV2_VAL"
upsert_env "ENABLE_FORMATION"          "$FORMATION_VAL"
upsert_env "FORMATION_SCALE"           "$FORMATION_SCALE_VAL"
upsert_env "MITIGATION"                "$MITIGATION_VAL"
upsert_env "RUN_TAG"                   "$RUN_TAG_VAL"
upsert_env "DUMMY"                     "$DUMMY_VAL"
# Deployed contract parameters, read back by the experiment runner.
upsert_env "JURY_SIZE"                 "${JURY_SIZE:-}"
upsert_env "JURY_THRESHOLD"            "${JURY_THRESHOLD:-}"
upsert_env "JURY_TIMEOUT_BLOCKS"       "${JURY_TIMEOUT_BLOCKS:-}"
upsert_env "FRESHNESS_BLOCKS"          "${FRESHNESS_BLOCKS:-}"

# Mission of the orchestrator for the selected world.
if [[ "$WORLD_VAL" == "swarm_arena" ]]; then
  upsert_env "MISSION_FILE" "/app/missions/swarm_arena.json"
else
  upsert_env "MISSION_FILE" "/app/missions/turtlebot3_world.json"
fi

CPU_DISPLAY="$CPU_LIMIT_VAL"
[[ "$NO_CPU_LIMIT_VAL" == "TRUE" ]] && CPU_DISPLAY="none (uncapped)"
echo "✅ .env updated (Robots: $N_ROBOTS_VAL, Mode: $DEPLOY_MODE, Contract: $CONTRACT_VAL, Auto: $AUTO_START_VAL, Rounds: $ROUNDS_VAL, SSP: ${SSP_VAL}ms ($SSP_MODE_VAL), CPU: $CPU_DISPLAY, IterQ: $ITERQ_VAL)"

# ── Compose files ─────────────────────────────────────────────────────────────
# Where sidecars reach Besu: this host (local) or its LAN IP (remote).
if [[ "$DEPLOY_MODE" == "remote" ]]; then
  BLOCKCHAIN_HOST="$LOCAL_IP"
else
  BLOCKCHAIN_HOST="host.docker.internal"
fi
echo "🔧 Generating compose files for $N_ROBOTS_VAL robot(s) [mode: $DEPLOY_MODE]..."
COMPOSE_FLAGS=(--robots "$N_ROBOTS_VAL" --blockchain-host "$BLOCKCHAIN_HOST" --mode "$DEPLOY_MODE" --contract "$CONTRACT_VAL" --world "$WORLD_VAL")
[[ "$FORMATION_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--formation --formation-scale "$FORMATION_SCALE_VAL")
[[ "$NO_CPU_LIMIT_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--no-cpu-limit)
[[ "$ATTEST_GZSERVER_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--attest-gzserver)
[[ "$GPU_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--gpu)
[[ "$GUI_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--gui)
[[ "$DUMMY_VAL" == "TRUE" ]] && COMPOSE_FLAGS+=(--dummy)
python3 "$SCRIPT_DIR/generate_compose.py" "${COMPOSE_FLAGS[@]}"
echo

# ── Event-watcher checkpoints ─────────────────────────────────────────────────
# The chain is new, so the last processed block saved by each watcher is stale.
# The files are root-owned (written by containers): delete them from a container.
CHECKPOINT_DIR="$SCRIPT_DIR/checkpoints"
RESET_SCRIPT="$CHECKPOINT_DIR/reset_event_watcher.sh"

if [[ -d "$CHECKPOINT_DIR" && -f "$RESET_SCRIPT" ]]; then
  echo "🧹 Resetting local EventWatcher checkpoints..."
  docker run --rm -v "$CHECKPOINT_DIR":/cp alpine sh -c 'find /cp -name "*.json" -delete'
  echo
else
  echo "⚠️  Skipping local checkpoint reset (missing $RESET_SCRIPT or $CHECKPOINT_DIR)"
  echo
fi

if [[ "$DEPLOY_MODE" == "remote" ]]; then
  echo "🧹 Resetting remote EventWatcher checkpoints on $REMOTE1_HOST..."
  run_remote "$REMOTE1_HOST" "$REMOTE1_USER" \
    "mkdir -p $REMOTE_DIR/checkpoints && docker run --rm -v \$(realpath $REMOTE_DIR/checkpoints):/cp alpine sh -c 'find /cp -name \"*.json\" -delete' && echo 'Done.'"
  echo
  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )) && [[ -n "$REMOTE2_HOST" ]]; then
    echo "🧹 Resetting remote EventWatcher checkpoints on $REMOTE2_HOST..."
    run_remote "$REMOTE2_HOST" "$REMOTE2_USER" \
      "mkdir -p $REMOTE_DIR/checkpoints && docker run --rm -v \$(realpath $REMOTE_DIR/checkpoints):/cp alpine sh -c 'find /cp -name \"*.json\" -delete' && echo 'Done.'"
    echo
  fi
fi

# ── Blockchain (always on this host) ──────────────────────────────────────────
cd "$SCRIPT_DIR/blockchain/quorum-test-network"
if [[ "$DASHBOARDS_VAL" == "TRUE" ]]; then
  COMPOSE_PROFILES=dashboards ./run.sh
else
  ./run.sh
fi

sleep 10

cd "$SCRIPT_DIR"
./deploy_sc.sh --rpc_url http://localhost:21001 --chain_id 1337 --contract "$CONTRACT_VAL"

# ── Agent configs ─────────────────────────────────────────────────────────────
# config/: Gazebo robots (+ placeholder references in ref-measurements/).
# config-dummy/: dummy-publisher robots (--dummy).
echo "🔧 Generating agent configs (contract: $CONTRACT_VAL, blockchain-host: $BLOCKCHAIN_HOST)..."

if [[ "$ATTEST_GZSERVER_VAL" == "TRUE" ]]; then
  _ATTEST_CMD_NAME="gzserver"
  _ATTEST_TEXT_SIZE="$GZSERVER_TEXT_SECTION_SIZE"
  _ATTEST_OFFSET="$GZSERVER_TEXT_SECTION_OFFSET"
elif [[ "$ATTEST_MISSION_AGENT_VAL" == "TRUE" ]]; then
  _ATTEST_CMD_NAME="mission_agent"
  _ATTEST_TEXT_SIZE="$MISSION_AGENT_TEXT_SECTION_SIZE"
  _ATTEST_OFFSET="$MISSION_AGENT_TEXT_SECTION_OFFSET"
elif [[ "$FORMATION_VAL" == "TRUE" ]]; then
  _ATTEST_CMD_NAME="formation_agent"
  _ATTEST_TEXT_SIZE="$FORMATION_AGENT_TEXT_SECTION_SIZE"
  _ATTEST_OFFSET="$FORMATION_AGENT_TEXT_SECTION_OFFSET"
else
  _ATTEST_CMD_NAME="robot_state_publisher"
  _ATTEST_TEXT_SIZE="42223"
  _ATTEST_OFFSET="0"
fi

CONFIG_ARGS=(
  --num-agents 100
  --contract "$CONTRACT_VAL"
  --config-output "$SCRIPT_DIR/config"
  --ref-output "$SCRIPT_DIR/ref-measurements"
  --blockchain-host "$BLOCKCHAIN_HOST"
  --secaas-host "host.docker.internal"
  --cmd-name "$_ATTEST_CMD_NAME"
  --text-section-size "$_ATTEST_TEXT_SIZE"
  --offset "$_ATTEST_OFFSET"
)
python3 "$SCRIPT_DIR/create_agent_config.py" "${CONFIG_ARGS[@]}"

python3 "$SCRIPT_DIR/create_agent_config.py" \
  --num-agents 100 \
  --contract "$CONTRACT_VAL" \
  --config-output "$SCRIPT_DIR/config-dummy" \
  --blockchain-host "$BLOCKCHAIN_HOST" \
  --cmd-name dummy_publisher \
  --text-section-size 175521 \
  --offset 5744

echo

wait_for_secaas() {
  echo "⏳ Waiting for SECaaS API to be ready at $SECAAS_URL..."
  local max_attempts=30
  local attempt=1
  while (( attempt <= max_attempts )); do
    if curl -s "$SECAAS_URL/" | grep -q "SECaaS running"; then
      echo "✅ SECaaS is UP and running!"
      return 0
    fi
    echo "   (Attempt $attempt/$max_attempts) Still waiting..."
    sleep 2
    (( attempt++ ))
  done
  echo "❌ SECaaS failed to start in time."
  return 1
}

# ── Containers ────────────────────────────────────────────────────────────────
if [[ "$DEPLOY_MODE" == "remote" ]]; then
  echo "🌐 Remote mode: d-mutra hosts Besu+SECaaS+monitoring; robots+sidecars on remote host(s)"
  echo

  echo "📤 Syncing to remote1 ($REMOTE1_HOST)..."
  scp "$ENV_FILE" "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/.env"
  rsync -av "$SCRIPT_DIR/generated/"    "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/generated/"
  rsync -av "$SCRIPT_DIR/config/"       "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/config/"
  rsync -av "$SCRIPT_DIR/config-dummy/" "$REMOTE1_USER@$REMOTE1_HOST:$REMOTE_DIR/config-dummy/"
  echo

  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "📤 Syncing to remote2 ($REMOTE2_HOST)..."
    scp "$ENV_FILE" "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/.env"
    rsync -av "$SCRIPT_DIR/generated/"    "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/generated/"
    rsync -av "$SCRIPT_DIR/config-dummy/" "$REMOTE2_USER@$REMOTE2_HOST:$REMOTE_DIR/config-dummy/"
    echo
  fi

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d

  if wait_for_secaas; then
    echo "🔄 Synchronizing agent signatures to PostgreSQL..."
    curl -X POST "$SECAAS_URL/sync-agents?n_robots=$N_ROBOTS_VAL" -H "Content-Type: application/json" | jq
    echo "✅ Sync complete."
  else
    exit 1
  fi

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

  echo "🤖 Starting robots on remote1 ($REMOTE1_HOST, robots 1–$(( N_ROBOTS_VAL < REMOTE1_LIMIT ? N_ROBOTS_VAL : REMOTE1_LIMIT )))..."
  if ! remote_compose_up "$REMOTE1_HOST" "$REMOTE1_USER" \
      "$REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote1.yml"; then
    echo "❌ Failed to start robots on remote1."
    exit 1
  fi

  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "🤖 Starting robots on remote2 ($REMOTE2_HOST, robots $((REMOTE1_LIMIT+1))–$N_ROBOTS_VAL)..."
    if ! remote_compose_up "$REMOTE2_HOST" "$REMOTE2_USER" \
        "$REMOTE_DIR/generated/docker-compose.robots-N${N_ROBOTS_VAL}-remote2.yml"; then
      echo "❌ Failed to start robots on remote2."
      exit 1
    fi
  fi

  echo "📊 Starting monitoring on remote1..."
  run_remote "$REMOTE1_HOST" "$REMOTE1_USER" \
    "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME \
     -f $REMOTE_DIR/docker-compose.monitoring.yml up -d"

  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "📊 Starting monitoring on remote2..."
    run_remote "$REMOTE2_HOST" "$REMOTE2_USER" \
      "cd $REMOTE_DIR && COMPOSE_IGNORE_ORPHANS=1 docker compose -p $PROJECT_NAME \
       -f $REMOTE_DIR/docker-compose.monitoring.yml up -d"
  fi

  echo "🛡️  Starting sidecars on remote1..."
  if ! remote_compose_up "$REMOTE1_HOST" "$REMOTE1_USER" \
      "$REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote1.yml"; then
    echo "❌ Failed to start sidecars on remote1."
    exit 1
  fi

  if (( N_ROBOTS_VAL > REMOTE1_LIMIT )); then
    echo "🛡️  Starting sidecars on remote2..."
    if ! remote_compose_up "$REMOTE2_HOST" "$REMOTE2_USER" \
        "$REMOTE_DIR/generated/docker-compose.attestation-N${N_ROBOTS_VAL}-remote2.yml"; then
      echo "❌ Failed to start sidecars on remote2."
      exit 1
    fi
  fi

else
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.robots.yml" up -d

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.secaas.yml" up -d

  if wait_for_secaas; then
    echo "🔄 Synchronizing agent signatures to PostgreSQL..."
    curl -X POST "$SECAAS_URL/sync-agents?n_robots=$N_ROBOTS_VAL" -H "Content-Type: application/json" | jq
    echo "✅ Sync complete."
  else
    exit 1
  fi

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.monitoring.yml" up -d

  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.attestation.yml" up -d

fi

# ── Reference measurements ────────────────────────────────────────────────────
# V2/V3: two admins approve each robot's reference on chain.
anchor_references() {
  [[ "$CONTRACT_VAL" == AttestationManagerV[23] ]] || return 0
  echo "⚓ Anchoring references on chain..."
  "$SCRIPT_DIR/set_references.sh" "$CONTRACT_VAL" "$N_ROBOTS_VAL" \
    "$ADMIN_NODES_DIR/validator2/accountPrivateKey" \
    "$ADMIN_NODES_DIR/validator3/accountPrivateKey"
}

# LV: clear the on-chain state through the SECaaS. V2/V3 are fresh, and only
# their admins could reset them.
reset_chain() {
  [[ "$CONTRACT_VAL" == AttestationManagerV[23] ]] && return 0
  echo "🧹 Resetting on-chain attestation chain..."
  curl -s -X POST "$SECAAS_URL/reset" | jq .
}

# Host serving robot <i>'s sidecar API (port 8000 + i).
sidecar_host() {
  if [[ "$DEPLOY_MODE" == "local" ]]; then
    echo "localhost"
  elif (( $1 <= REMOTE1_LIMIT )); then
    echo "$REMOTE1_HOST"
  else
    echo "$REMOTE2_HOST"
  fi
}

# Every robot runs the same binary and sidecar, so robot1's measured hashes
# (/digest) become every robot's reference, in the SECaaS database and, for
# V2/V3, on chain. Then start the workers if --auto.
if [[ "$BOOTSTRAP_REFS" == "TRUE" ]]; then
  SOURCE_HOST="$(sidecar_host 1)"
  echo
  echo "🔧 Bootstrapping real reference measurements (robot1 on $SOURCE_HOST)..."

  # /digest answers once the attested process runs (~30 s with Gazebo).
  max_attempts=70   # ~2 min
  attempt=1
  while (( attempt <= max_attempts )); do
    if curl -s "http://$SOURCE_HOST:8001/digest" 2>/dev/null | grep -q '"combined_hash"'; then
      echo "✅ robot1 /digest is ready."
      break
    fi
    echo "   (Attempt $attempt/$max_attempts) Waiting for /digest..."
    sleep 2
    (( attempt++ ))
  done
  if (( attempt > max_attempts )); then
    echo "⚠️  /digest didn't come up in time. Skipping bootstrap — attestations will FAIL on placeholder refs until you run bootstrap_ref_measurements.py manually."
  else
    python3 "$SCRIPT_DIR/bootstrap_ref_measurements.py" --robots "$N_ROBOTS_VAL" \
        --sidecar-host "$SOURCE_HOST" --sync-secaas "${BOOTSTRAP_FLAGS[@]}" \
      && anchor_references \
      && reset_chain \
      || echo "⚠️  Bootstrap failed — attestations may close as FAILURE."
  fi

  # .env records what was asked; the containers booted idle either way.
  upsert_env "AUTO_START" "$WANT_AUTO_START"

  if [[ "$WANT_AUTO_START" == "TRUE" ]]; then
    echo "▶ Auto-starting attestation workers (SECaaS + sidecars)..."
    curl -s -X POST "$SECAAS_URL/start" | jq .
    for i in $(seq 1 "$N_ROBOTS_VAL"); do
      curl -s -X POST "http://$(sidecar_host "$i"):$((8000 + i))/start" | jq -c .
    done
  fi
fi

# ── Orchestrators (started last, once the robot agents are up) ────────────────
# Both watch the chain for attestation FAILUREs. Mission: reassigns the failed
# robot's waypoints. Formation: isolates the failed robot. Only with --mitigation on.
if [[ "$MISSION_VAL" == "TRUE" && "$DEPLOY_MODE" == "local" ]]; then
  echo
  echo "🧭 Starting mission orchestrator (mitigation=$MITIGATION_VAL, run-tag=$RUN_TAG_VAL)..."
  mkdir -p "$SCRIPT_DIR/experiments/data/mission"
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.orchestrator.yml" up -d
  echo "   Logs will appear in experiments/data/mission/mission-${RUN_TAG_VAL}.csv"
fi

if [[ "$FORMATION_VAL" == "TRUE" && "$DEPLOY_MODE" == "local" ]]; then
  echo
  echo "🛰️  Starting formation orchestrator (mitigation=$MITIGATION_VAL, scale=$FORMATION_SCALE_VAL, run-tag=$RUN_TAG_VAL)..."
  FDATA_SUBDIR="${FORMATION_DATA_SUBDIR:-formation}"
  mkdir -p "$SCRIPT_DIR/experiments/data/$FDATA_SUBDIR"
  mkdir -p "$SCRIPT_DIR/experiments/data/$FDATA_SUBDIR/bags"
  docker compose -p "$PROJECT_NAME" -f "$SCRIPT_DIR/docker-compose.formation.yml" up -d
  echo "   Logs will appear in experiments/data/$FDATA_SUBDIR/formation-${RUN_TAG_VAL}.csv"
fi

echo "🎉 All done."
