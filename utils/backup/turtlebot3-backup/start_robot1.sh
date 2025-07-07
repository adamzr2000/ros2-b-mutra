#!/bin/bash
# Initialise ROS2 + workspace + pyenv
source /opt/ros/${ROS_DISTRO}/setup.bash
source /home/agent/ros2_ws/install/setup.bash

# Active pyenv localement
export PYENV_ROOT="/home/agent/.pyenv"
export PATH="$PYENV_ROOT/bin:$PATH"
eval "$(pyenv init --path)"
eval "$(pyenv init -)"
eval "$(pyenv virtualenv-init -)"
cd /home/agent/scripts
pyenv local dlt-env

# Démarre le binaire ROS
exec /home/agent/ros2_ws/build/robot_state_publisher/robot_state_publisher /home/agent/ros2_ws/src/robot_state_publisher/urdf/test-desc.urdf
