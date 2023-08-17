FROM ros:noetic-ros-base

# Base dependencies
RUN apt update \
    && apt install -y \
    python3-catkin-tools \
    python3-dev \
    python3-pip \
    python3-catkin-pkg-modules \
    python3-numpy \
    python3-yaml \
    ros-noetic-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
