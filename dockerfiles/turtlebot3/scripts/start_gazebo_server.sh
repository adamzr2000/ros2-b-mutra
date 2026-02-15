#!/bin/bash

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/agent/ros2_ws/install/setup.bash

echo "Starting Gazebo Server"

ros2 launch turtlebot3_gazebo gazebo_server.launch.py verbose:=true