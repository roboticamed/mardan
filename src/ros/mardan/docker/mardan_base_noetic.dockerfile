FROM ros:noetic-ros-base

# Base dependencies
RUN apt update \
    && apt install -y \
    tmux \
    python3-catkin-tools \
    python3-dev \
    python3-pip \
    python3-catkin-pkg-modules \
    python3-numpy \
    python3-smbus \
    python3-yaml \
    ros-noetic-cv-bridge \
    ros-noetic-teleop-twist-keyboard \
    && rm -rf /var/lib/apt/lists/*
