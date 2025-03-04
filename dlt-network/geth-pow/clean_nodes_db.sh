#!/bin/bash

# Define the directory pattern to match (e.g., node1/geth, node2/geth, etc.)
pattern="config/node*/geth"

# Find all matching directories and remove them
matches=($pattern)

if [ ${#matches[@]} -gt 0 ]; then
  for dir in "${matches[@]}"; do
    if [ -d "$dir" ]; then
      echo "Removing $dir..."
      rm -r "$dir"
    fi
  done
else
  echo "No matching directories found."
fi

# Remove all nodeX.log files
for file in logs/node*.log; do
  if [ -f "$file" ]; then
    echo "Removing file: $file"
    rm -f "$file"
  fi
done