#!/usr/bin/env bash

echo "Cleaning JSON and CSV files under current directory: $(pwd)"

# Show what will be deleted (preview)
echo "Files to be deleted:"
find . -type f \( -name "*.json" -o -name "*.csv" \)

# Ask for confirmation
read -p "Proceed with deletion? (y/N): " confirm
if [[ "$confirm" != "y" && "$confirm" != "Y" ]]; then
    echo "Aborted."
    exit 0
fi

# Delete files
find . -type f \( -name "*.json" -o -name "*.csv" \) -delete

echo "Cleanup complete."
