#!/bin/bash

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/agent/ros2_ws/install/setup.bash

export TURTLEBOT3_MODEL=${TURTLEBOT3_MODEL:-burger}
export NAMESPACE=${NAMESPACE:-""}
export X_POSE=${X_POSE:-0.0}
export Y_POSE=${Y_POSE:-0.0}
export ENABLE_NAV2=${ENABLE_NAV2:-false}
export ENABLE_FORMATION=${ENABLE_FORMATION:-false}
export FORMATION_SCALE=${FORMATION_SCALE:-2.0}
export K_ATT=${K_ATT:-0.8}
export K_FORM=${K_FORM:-3.0}

# Stagger startup so robots don't all call spawn_entity simultaneously.
# Gazebo's spawn service is sequential; concurrent calls from N>4 robots
# cause some to fail. Each robot waits (N-1)*3 s before proceeding.
_ROBOT_NUM=$(echo "${NAMESPACE}" | grep -oE '[0-9]+$')
_STAGGER_S=$(( (${_ROBOT_NUM:-1} - 1) * 5 ))
if (( _STAGGER_S > 0 )); then
    echo "Staggering spawn by ${_STAGGER_S}s (${NAMESPACE})..."
    sleep "${_STAGGER_S}"
fi
unset _ROBOT_NUM _STAGGER_S

# NOTE: The mission_agent drives the robot directly with velocity commands using
# ground-truth odometry (no Nav2/AMCL). There is therefore no AMCL initial-pose
# injection step — the agent becomes "ready" as soon as it receives /odom.

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
echo "Nav2:  $ENABLE_NAV2"

# Launch the robot (state publisher + spawn + mission_agent when enabled).
ros2 launch turtlebot3_gazebo robot.launch.py \
    namespace:=$NAMESPACE \
    x_pose:=$X_POSE \
    y_pose:=$Y_POSE \
    spawn_timeout:=300 \
    enable_nav2:=$ENABLE_NAV2 \
    enable_formation:=$ENABLE_FORMATION \
    formation_scale:=$FORMATION_SCALE \
    k_att:=$K_ATT \
    k_form:=$K_FORM &
LAUNCH_PID=$!

wait $LAUNCH_PID