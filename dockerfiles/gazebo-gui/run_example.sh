#!/bin/bash

host_dir="$(pwd)/scripts"

docker run \
    --name gazebo-gui \
    --hostname gazebo-gui \
    --rm \
    -d \
    -p 6080:80 \
    --security-opt \
    seccomp=unconfined \
    -v ${host_dir}:/home/ubuntu/scripts \
    gazebo-gui:latest 

echo "Done."

