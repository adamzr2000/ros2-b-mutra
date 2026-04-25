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
waited=0
until ros2 node list | grep -q "/gazebo"; do
    sleep 1
    waited=$((waited + 1))
    if (( waited >= 300 )); then echo "❌ Timeout waiting for /gazebo node"; exit 1; fi
done
echo "/gazebo node has started!"

# Wait for spawn service exposed by Gazebo ROS factory plugin.
echo "Waiting for /spawn_entity service to start..."
waited=0
until ros2 service list | grep -q "^/spawn_entity$"; do
    sleep 1
    waited=$((waited + 1))
    if (( waited >= 300 )); then echo "❌ Timeout waiting for /spawn_entity service"; exit 1; fi
done
echo "/spawn_entity service is available!"

# Print debug info
echo "Launching TurtleBot3 with the following parameters:"
echo "Namespace: $NAMESPACE"
echo "X Pose: $X_POSE"
echo "Y Pose: $Y_POSE"

# Launch command
ros2 launch turtlebot3_gazebo robot.launch.py namespace:=$NAMESPACE x_pose:=$X_POSE y_pose:=$Y_POSE spawn_timeout:=300