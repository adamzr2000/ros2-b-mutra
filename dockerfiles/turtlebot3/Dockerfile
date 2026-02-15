ARG ROS_DISTRO=humble
FROM osrf/ros:${ROS_DISTRO}-desktop

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# System deps
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    git \
    python3 \
    python3-pip \
    iputils-ping \
    build-essential \
    lsb-release \
    ros-${ROS_DISTRO}-rmw-cyclonedds-cpp \
    ros-${ROS_DISTRO}-rosidl-generator-dds-idl \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    rm -rf /var/lib/apt/lists/*

# Remove pre-installed packages to avoid conflicts
RUN apt-get remove -y ros-${ROS_DISTRO}-robot-state-publisher || true
# RUN apt-get remove -y ros-${ROS_DISTRO}-gazebo-ros-pkgs || true


# Create a new user 'agent' with sudo privileges and set a password
RUN useradd -m agent && \
    echo "agent:agent" | chpasswd && \
    adduser agent sudo

USER agent
WORKDIR /home/agent

# Copy workspace
RUN mkdir -p /home/agent/ros2_ws/src /home/agent/scripts
COPY --chown=agent:agent src/ /home/agent/ros2_ws/src/
COPY --chown=agent:agent scripts/ /home/agent/scripts/

# Build
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    cd /home/agent/ros2_ws && \
    colcon build --symlink-install

# Switch to user for runtime
RUN chown -R agent:agent /home/agent
USER agent

# Source overlays in bash sessions
RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> /home/agent/.bashrc && \
    echo "source /home/agent/ros2_ws/install/setup.bash" >> /home/agent/.bashrc

WORKDIR /home/agent/scripts/

CMD ["/bin/bash"]