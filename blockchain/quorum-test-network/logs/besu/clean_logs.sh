#!/bin/bash

echo "Starting cleanup of besu logs..."

# Remove standard logs and compressed logs
# The -f flag forces removal without prompting for each file
rm -f besu-*.log besu-*.log.gz

echo "Cleanup complete! All besu log files have been removed."
