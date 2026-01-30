#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

for d in "attestation-times" "docker-stats"; do
  results_dir="$BASE_DIR/$d/results"

  echo "Cleaning: $results_dir"

  if [[ -d "$results_dir" ]]; then
    rm -f -- "$results_dir"/* 2>/dev/null || true
  else
    echo "  (skipped: no results dir)"
  fi
done

echo "Done."
