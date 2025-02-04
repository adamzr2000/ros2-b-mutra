#!/bin/bash

# Wait for /gazebo node to start
echo "Waiting for /gazebo node to start..."
until ros2 node list | grep -q "/gazebo"; do
    sleep 1
done
echo "/gazebo node has started!"

# Set default values if environment variables are not set
export NAMESPACE=${NAMESPACE:-""}
export X_POSE=${X_POSE:-0.0}
export Y_POSE=${Y_POSE:-0.0}

# Print debug info
echo "Launching TurtleBot3 with the following parameters:"
echo "Namespace: $NAMESPACE"
echo "X Pose: $X_POSE"
echo "Y Pose: $Y_POSE"

# Set TurtleBot3 model
export TURTLEBOT3_MODEL=burger

# Source ROS2 workspace
source /home/agent/ros2_ws/install/setup.bash

# Launch the TurtleBot3 Gazebo simulation with environment-configured parameters
ros2 launch turtlebot3_gazebo robot.launch.py namespace:=$NAMESPACE x_pose:=$X_POSE y_pose:=$Y_POSE
