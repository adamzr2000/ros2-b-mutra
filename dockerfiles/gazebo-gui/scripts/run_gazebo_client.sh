#!/bin/bash

echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

source /opt/ros/humble/setup.bash

# Prepend the mounted models path AFTER sourcing setup.bash, which may set or
# append its own paths to GAZEBO_MODEL_PATH. Using unconditional prepend (not :-
# fallback) ensures the bind-mounted models are always found first, regardless of
# what setup.bash did to the variable.
export GAZEBO_MODEL_PATH="/home/ubuntu/turtlebot3_models:${GAZEBO_MODEL_PATH}"

ros2 launch gazebo_ros gzclient.launch.py