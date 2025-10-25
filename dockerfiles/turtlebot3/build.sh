#!/bin/bash
IMAGE_NAME="turtlebot3-simulation-ros2"

echo 'Building docker image.'

docker build -t $IMAGE_NAME .
