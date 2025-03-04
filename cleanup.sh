#!/bin/bash

# Function to delete a file if it exists
function delete_file() {
    if [ -f "$1" ]; then
        rm -f "$1"  # Add the -f flag to suppress confirmation prompts
        echo "Deleted: $1"
    fi
}

# Delete JSON files in the current directory
for file in ./config/*.json; do
    delete_file "$file"
done

# Delete .env files and docker-compose.yml files in the current directory
delete_file "docker-compose.yml"