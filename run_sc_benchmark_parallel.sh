#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="${IMAGE:-attestation-sidecar:latest}"
OUT_DIR_HOST="${OUT_DIR_HOST:-${ROOT_DIR}/experiments/data/sc-benchmark}"
CONFIG_DIR_HOST="${CONFIG_DIR_HOST:-${ROOT_DIR}/config}"

RUNS="${RUNS:-5}"
TIMEOUT="${TIMEOUT:-60}"
CONCURRENCY="${CONCURRENCY:-10}"
ACCOUNTS="${ACCOUNTS:-}"
FUNCTIONS="${FUNCTIONS:-RegisterAgent,SendEvidence,SendRefSignature,CloseAttestationProcess}"

mkdir -p "${OUT_DIR_HOST}"

if [[ ! -d "${CONFIG_DIR_HOST}" ]]; then
	echo "Config directory not found: ${CONFIG_DIR_HOST}" >&2
	exit 1
fi

echo "Running parallel smart-contract benchmark in container..."
echo "- Image:        ${IMAGE}"
echo "- Config Dir:   ${CONFIG_DIR_HOST}"
echo "- Output:       ${OUT_DIR_HOST}"
echo "- Accounts:     ${ACCOUNTS:-all}"
echo "- Concurrency:  ${CONCURRENCY}"
echo "- Runs:         ${RUNS}"
echo "- Timeout:      ${TIMEOUT}"
echo ""

DOCKER_ARGS=(
	"--rm"
	"--network" "quorum-dev-quickstart"
	"-v" "${OUT_DIR_HOST}:/out"
	"-v" "${CONFIG_DIR_HOST}:/config:ro"
	"--entrypoint" "./txbench-parallel"
	"${IMAGE}"
	"-config-dir" "/config"
	"-out" "/out"
	"-runs" "${RUNS}"
	"-timeout" "${TIMEOUT}"
	"-concurrency" "${CONCURRENCY}"
	"-functions" "${FUNCTIONS}"
)

if [[ -n "${ACCOUNTS}" ]]; then
	DOCKER_ARGS+=("-accounts" "${ACCOUNTS}")
fi

docker run "${DOCKER_ARGS[@]}" "$@"
