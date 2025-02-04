#!/bin/bash

echo "Using ROS_DOMAIN_ID: ${ROS_DOMAIN_ID}"

source /opt/ros/humble/setup.bash

ros2 launch gazebo_ros gzclient.launch.py