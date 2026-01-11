#!/usr/bin/env bash
# remove all json files under the current directory (recursively)

set -euo pipefail

echo "Removing JSONs under: $(pwd)"
find . -type f -iname '*.json' -print -delete

echo "Removing CSVs under: $(pwd)"
find . -type f -iname '*.csv' -print -delete

echo "Done."

