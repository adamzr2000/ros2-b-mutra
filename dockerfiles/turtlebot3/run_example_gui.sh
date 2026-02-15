#!/bin/bash

# Allow X11 connections
xhost +local:docker

# XAUTH=/tmp/.docker.xauth
XAUTH=$HOME/.Xauthority

host_dir="./src"

container_image="turtlebot3-gazebo"

container_name=$container_image

docker run \
    -it \
    --name $container_name \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --volume="/tmp/.X11-unix:/tmp/.X11-unix:rw" \
    --env="XAUTHORITY=$XAUTH" \
    --volume="$XAUTH:$XAUTH" \
    -v ${host_dir}:/home/agent/ros2_ws/src \
    --rm \
    --net host \
    --privileged \
    $container_image:latest

# Revoke access to the X server after the container exits
xhost -local:docker

echo "Done."