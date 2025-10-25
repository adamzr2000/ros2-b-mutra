#!/bin/bash

# Default container name
container_name="ros2-agent"

# Parse options
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --name)
            container_name="$2"
            shift 2
            ;;
        *)
            echo "Unknown option: $1"
            exit 1
            ;;
    esac
done

# Host directory
host_dir="./ros2_ws/src"

# Run the container
docker run \
    -it \
    --name "${container_name}" \
    --hostname "${container_name}" \
    -v "${host_dir}:/home/agent/ros2_ws/src" \
    --rm \
    --privileged \
    turtlebot3-simulation-ros2:latest

echo "Done."
