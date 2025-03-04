# Specify the ROS2 distribution as a build argument
ARG ROS_DISTRO=humble

# Use an official ROS base image
FROM osrf/ros:${ROS_DISTRO}-desktop

# Define maintainer
LABEL maintainer="azahir@pa.uc3m.es"

# Set up user with sudo access
WORKDIR /root

# Create a new user 'agent' with sudo privileges and set a password
RUN useradd -m agent && \
    echo "agent:agent" | chpasswd && \
    adduser agent sudo

# Set an environment variable to noninteractive to avoid prompts during package installation
ENV DEBIAN_FRONTEND=noninteractive

# Use bash shell for subsequent commands
SHELL [ "/bin/bash" , "-c" ]

# Remove pre-installed packages to avoid conflicts
RUN apt-get remove -y ros-${ROS_DISTRO}-robot-state-publisher || true
# RUN apt-get remove -y ros-${ROS_DISTRO}-gazebo-ros-pkgs || true

# Switch to the newly created user for better security (avoid using root)
USER agent
WORKDIR /home/agent

# Clone the necessary ROS packages into the workspace
RUN mkdir -p ros2_ws/src 
COPY ros2_ws/src /home/agent/ros2_ws/src

RUN mkdir scripts && mkdir smart-contracts && mkdir config

USER root

# Install necessary packages and cleaning up the apt cache to reduce the image size
RUN apt-get update && apt-get install -y \
    curl \
    git \
    python3 \
    python3-pip \
    iputils-ping \
    build-essential \
    lsb-release \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs && \
    rm -rf /var/lib/apt/lists/*

# RUN cd /home/agent/ros2_ws && \
#     rosdep update && \
#     rosdep install --from-paths src --ignore-src -r -y

USER agent
WORKDIR /home/agent

# Install pyenv as 'agent' user
RUN curl https://pyenv.run | bash

# Configure pyenv for all users
RUN echo 'export PATH="$HOME/.pyenv/bin:$PATH"' >> ~/.bashrc && \
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc && \
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init --path)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc

# Ensure pyenv is properly sourced before usage
RUN export PATH="$HOME/.pyenv/bin:$PATH" && \
    eval "$(pyenv init --path)" && \
    eval "$(pyenv init -)" && \
    pyenv install 3.8.10 && \
    pyenv global 3.8.10


# Build the workspace
RUN cd ros2_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/agent/ros2_ws/install/setup.bash" >> ~/.bashrc

COPY scripts/* /home/agent/scripts/

WORKDIR /home/agent/scripts/

# Install dependencies for agent user
RUN export PATH="/home/agent/.pyenv/versions/3.8.10/bin:$PATH" && \
    /home/agent/.pyenv/versions/3.8.10/bin/python3 -m pip install --no-cache-dir --upgrade pip && \
    /home/agent/.pyenv/versions/3.8.10/bin/python3 -m pip install --no-cache-dir -r requirements.txt


# Set default user to 'agent' and ensure Python and pip are always correct
USER agent
ENV PATH="/home/agent/.pyenv/versions/3.8.10/bin:$PATH"
ENV PYENV_ROOT="/home/agent/.pyenv"
ENV PYTHONPATH="/home/agent/.pyenv/versions/3.8.10/lib/python3.8/site-packages"

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"

