#!/bin/bash

# Source ROS2 workspace
source /opt/ros/humble/setup.bash
source /home/agent/ros2_ws/install/setup.bash

WORLDS_DIR="$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/worlds"

case "${GAZEBO_WORLD:-empty}" in
  turtlebot3_world) WORLD_FILE="$WORLDS_DIR/turtlebot3_world.world" ;;
  swarm_arena)      WORLD_FILE="$WORLDS_DIR/swarm_arena.world" ;;
  house)            WORLD_FILE="$WORLDS_DIR/turtlebot3_house.world" ;;
  dqn1)             WORLD_FILE="$WORLDS_DIR/turtlebot3_dqn_stage1.world" ;;
  dqn2)             WORLD_FILE="$WORLDS_DIR/turtlebot3_dqn_stage2.world" ;;
  dqn3)             WORLD_FILE="$WORLDS_DIR/turtlebot3_dqn_stage3.world" ;;
  dqn4)             WORLD_FILE="$WORLDS_DIR/turtlebot3_dqn_stage4.world" ;;
  *)                WORLD_FILE="$WORLDS_DIR/empty_world.world" ;;
esac

echo "Starting Gazebo Server (world: ${GAZEBO_WORLD:-empty})"

ros2 launch turtlebot3_gazebo gazebo_server.launch.py verbose:=true world:="$WORLD_FILE"