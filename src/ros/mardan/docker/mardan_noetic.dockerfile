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

WORKDIR /opt/src
COPY /src/ros/mardan /opt/src/catkin_ws/src/mardan
SHELL ["/bin/bash", "-c"]
RUN source /opt/ros/noetic/setup.bash \
    && cd /opt/src/catkin_ws/ \
    && catkin_make -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic install \
    && rm -rf /opt/src/catkin_ws

SHELL ["/bin/bash", "-c"]
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
