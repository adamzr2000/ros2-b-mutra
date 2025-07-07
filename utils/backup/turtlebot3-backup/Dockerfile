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

RUN mkdir scripts && mkdir smart-contracts && mkdir config && mkdir ros2-scripts

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

RUN pip3 install --no-cache-dir numpy 
 
# Build the workspace
RUN cd ros2_ws && source /opt/ros/${ROS_DISTRO}/setup.bash && colcon build --symlink-install

RUN echo "source /opt/ros/${ROS_DISTRO}/setup.bash" >> ~/.bashrc
RUN echo "source /home/agent/ros2_ws/install/setup.bash" >> ~/.bashrc

COPY ros2-scripts/* /home/agent/ros2-scripts/
COPY scripts/* /home/agent/scripts/

# Install pyenv and ensure it's properly initialized for both interactive and non-interactive shells
RUN curl https://pyenv.run | bash && \
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.bashrc && \
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init --path)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv init -)"' >> ~/.bashrc && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.bashrc && \
    echo 'export PYENV_ROOT="$HOME/.pyenv"' >> ~/.profile && \
    echo 'export PATH="$PYENV_ROOT/bin:$PATH"' >> ~/.profile && \
    echo 'eval "$(pyenv init --path)"' >> ~/.profile && \
    echo 'eval "$(pyenv init -)"' >> ~/.profile && \
    echo 'eval "$(pyenv virtualenv-init -)"' >> ~/.profile

# Explicitly set environment variables for pyenv in Docker
ENV PYENV_ROOT="/home/agent/.pyenv"
ENV PATH="$PYENV_ROOT/bin:$PATH"


# Install only Python 3.8.10 using pyenv
RUN pyenv install 3.8.10

# Create a virtual environment inside /home/agent/scripts/
RUN pyenv virtualenv 3.8.10 dlt-env

WORKDIR /home/agent/scripts/

# Set the virtual environment locally in /home/agent/scripts/
RUN pyenv local dlt-env

# Force the shell to recognize the virtualenv before installing packages
RUN /home/agent/.pyenv/versions/dlt-env/bin/pip install --no-cache-dir -r requirements.txt

COPY start_robot1.sh .
# COPY sidecar_pause_on_process.sh .
# RUN chmod +x sidecar_pause_on_process.sh

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"
