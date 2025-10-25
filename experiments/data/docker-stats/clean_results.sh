#!/usr/bin/env bash
# remove all CSV files under the current directory (recursively)

set -euo pipefail

echo "Removing CSVs under: $(pwd)"
find . -type f -iname '*.csv' -print -delete
echo "Done."

