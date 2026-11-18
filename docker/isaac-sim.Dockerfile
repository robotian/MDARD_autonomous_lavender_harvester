# Isaac Sim + a system ROS 2 Jazzy install.
# The ROS 2 libraries bundled with Isaac Sim only contain Fast DDS and Cyclone DDS. To use rmw_zenoh_cpp (the
# middleware of the real robots) Isaac Sim has to load a system ROS 2 instead: it does so when a distribution
# is sourced (see isaacsim.ros2.core docs, "ROS_DISTRO"), which docker/isaac-entrypoint.sh does on demand.
# ISAAC_TAG: the fleet runs 6.0.0 (docker-compose.yml); others can be tried, e.g.
# docker build --build-arg ISAAC_TAG=6.2.0 -t a300-isaac-sim:6.2.0 -f docker/isaac-sim.Dockerfile docker
ARG ISAAC_TAG=6.0.0
FROM nvcr.io/nvidia/isaac-sim:${ISAAC_TAG}

USER root
ENV DEBIAN_FRONTEND=noninteractive
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" \
        > /etc/apt/sources.list.d/ros2.list \
    && apt-get update && apt-get install -y --no-install-recommends \
        ros-jazzy-ros-base \
        ros-jazzy-rmw-zenoh-cpp \
    && rm -rf /var/lib/apt/lists/*
USER 1234

COPY --chmod=755 isaac-entrypoint.sh /isaac-entrypoint.sh
ENTRYPOINT ["/isaac-entrypoint.sh"]
