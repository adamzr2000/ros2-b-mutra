ARG ROS_DISTRO=humble
FROM ros:${ROS_DISTRO}-ros-base

LABEL maintainer="azahir@pa.uc3m.es"

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ROS message packages used on the robot interfaces + pip for web3.
RUN apt-get update && apt-get install -y --no-install-recommends \
    python3-pip \
    ros-${ROS_DISTRO}-nav-msgs \
    ros-${ROS_DISTRO}-geometry-msgs \
    ros-${ROS_DISTRO}-std-msgs && \
    rm -rf /var/lib/apt/lists/*

COPY requirements.txt /tmp/requirements.txt
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

COPY app/ /app/
WORKDIR /app

ENV RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Source ROS and run the orchestrator. All behaviour is driven by env vars
# (see orchestrator.py header).
CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && exec python3 /app/orchestrator.py"]
