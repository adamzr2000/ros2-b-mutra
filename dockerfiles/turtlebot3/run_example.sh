#!/bin/bash

container_image="turtlebot3-gazebo"

container_name=$container_image

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
host_dir="./src"

# Run the container
docker run \
    -it \
    --name "${container_name}" \
    -v "${host_dir}:/home/agent/ros2_ws/src" \
    --rm \
    --privileged \
    $container_image:latest

echo "Done."
