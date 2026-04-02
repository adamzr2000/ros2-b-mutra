#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

IMAGE="${IMAGE:-attestation-sidecar:latest}"
OUT_DIR_HOST="${OUT_DIR_HOST:-${ROOT_DIR}/experiments/data/sc-benchmark}"
CONFIG_HOST="${CONFIG_HOST:-${ROOT_DIR}/config/secaas.json}"

RUNS="${RUNS:-30}"
TIMEOUT="${TIMEOUT:-60}"
AGENT_NAME="${AGENT_NAME:-txbench-agent}"
FUNCTIONS="${FUNCTIONS:-RegisterAgent,SendEvidence,SendRefSignature,CloseAttestationProcess}"

mkdir -p "${OUT_DIR_HOST}"

if [[ ! -f "${CONFIG_HOST}" ]]; then
	echo "Config file not found: ${CONFIG_HOST}" >&2
	exit 1
fi

echo "Running smart-contract benchmark in container..."
echo "- Image:   ${IMAGE}"
echo "- Config:  ${CONFIG_HOST}"
echo "- Output:  ${OUT_DIR_HOST}"

docker run --rm \
	--network quorum-dev-quickstart \
	-v "${OUT_DIR_HOST}:/out" \
	-v "${CONFIG_HOST}:/config.json:ro" \
	--entrypoint ./txbench \
	"${IMAGE}" \
	-config /config.json \
	-out /out \
	-runs "${RUNS}" \
	-timeout "${TIMEOUT}" \
	-agent-name "${AGENT_NAME}" \
	-functions "${FUNCTIONS}" \
	"$@"

