FROM tiryoh/ros2-desktop-vnc:humble

ENV DEBIAN_FRONTEND=noninteractive
SHELL [ "/bin/bash" , "-c" ]

# Install necessary packages and cleaning up the apt cache to reduce the image size
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-turtlebot3-msgs \
    && rm -rf /var/lib/apt/lists/*


RUN mkdir -p /home/ubuntu/scripts
COPY scripts/* /home/ubuntu/scripts/
RUN chmod +x /home/ubuntu/scripts/*.sh

# RUN sed -i '/ROS_DOMAIN_ID/d' /home/ubuntu/.bashrc

# Set the working directory to where your scripts are
WORKDIR /home/ubuntu/scripts/

# Indicate completion of Dockerfile setup
RUN echo "ALL Done"