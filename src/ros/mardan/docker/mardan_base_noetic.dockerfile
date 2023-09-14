FROM ros:noetic-ros-base

# Base dependencies
RUN apt update \
    && apt install -y \
    net-tools \
    inetutils-ping \
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
    ros-noetic-image-view \
    && rm -rf /var/lib/apt/lists/*

# some weird problem with currently installed dependencies
RUN rm -rf /usr/local/lib/python3.8/dist-packages/OpenSSL/ \
    && rm -rf /usr/local/lib/python3.8/dist-packages/pyOpenSSL-23.2.0.dist-info/

RUN python3 -m pip install \ 
    pyopenssl \
    aiohttp \
    aiortc

ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
