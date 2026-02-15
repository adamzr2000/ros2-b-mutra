#!/bin/bash

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/agent/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export NAMESPACE=${NAMESPACE:-""}
export X_POSE=${X_POSE:-0.0}
export Y_POSE=${Y_POSE:-0.0}

# Wait for /gazebo node to start
echo "Waiting for /gazebo node to start..."
until ros2 node list | grep -q "/gazebo"; do
    sleep 1
done
echo "/gazebo node has started!"

# Print debug info
echo "Launching TurtleBot3 with the following parameters:"
echo "Namespace: $NAMESPACE"
echo "X Pose: $X_POSE"
echo "Y Pose: $Y_POSE"

# Launch command
ros2 launch turtlebot3_gazebo robot.launch.py namespace:=$NAMESPACE x_pose:=$X_POSE y_pose:=$Y_POSE