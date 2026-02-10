#!/usr/bin/env bash
set -euo pipefail

BASE_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

KEEP_NAMES=(.gitkeep)

for d in "attestation-times" "docker-stats"; do
  for sub in "results" "_summary"; do
    target_dir="$BASE_DIR/$d/$sub"
    echo "Cleaning: $target_dir"

    if [[ -d "$target_dir" ]]; then
      # Remove everything inside target_dir EXCEPT files listed in KEEP_NAMES.
      # Works for files and subdirectories, and handles hidden entries too.
      find "$target_dir" -mindepth 1 -maxdepth 1 \
        $(printf '! -name %q ' "${KEEP_NAMES[@]}") \
        -exec rm -rf -- {} + 2>/dev/null || true
    else
      echo "  (skipped: no dir)"
    fi
  done
done

echo "Done."

