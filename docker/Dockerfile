# syntax=docker/dockerfile:1
ARG ROS_DISTRO=lyrical
ARG UBUNTU_CODENAME=resolute
FROM ros:${ROS_DISTRO}-ros-base-${UBUNTU_CODENAME}

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-o", "pipefail", "-c"]

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    git \
    python3-colcon-common-extensions \
    python3-pip \
    python3-rosdep \
    python3-vcstool \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace/Ros2Sample

COPY scripts ./scripts
RUN chmod +x ./scripts/*.sh

CMD ["bash", "-lc", "source /opt/ros/${ROS_DISTRO}/setup.bash && ./scripts/build.sh && bash"]
