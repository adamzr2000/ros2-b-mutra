# Headless Gazebo server (gzserver + gazebo_ros plugins) shared by all robots.
#
# The worlds and models come from the turtlebot3_gazebo package kept in
# robot-simulation/, passed as a named build context (see dockerfiles/Makefile):
#   docker build -t gazebo-server \
#     --build-context turtlebot3_gazebo=./robot-simulation/src/turtlebot3_simulations/turtlebot3_gazebo \
#     ./gazebo-server
#
# Same base image as robot-simulation, so the two share their base layers.
FROM osrf/ros:humble-desktop

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ros2_ws
COPY --from=turtlebot3_gazebo . src/turtlebot3_gazebo/
RUN source /opt/ros/${ROS_DISTRO}/setup.bash && \
    colcon build --packages-select turtlebot3_gazebo

COPY scripts/start_gazebo_server.sh /usr/local/bin/start_gazebo_server.sh

CMD ["start_gazebo_server.sh"]
