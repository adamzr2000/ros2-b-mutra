#!/bin/bash

host_dir="$(pwd)/scripts"

docker run \
    --name gazebo-vnc \
    --hostname gazebo-vnc \
    --rm \
    -d \
    -p 6080:80 \
    --security-opt \
    seccomp=unconfined \
    -v ${host_dir}:/home/ubuntu/scripts \
    gazebo-vnc:latest 

echo "Done."

