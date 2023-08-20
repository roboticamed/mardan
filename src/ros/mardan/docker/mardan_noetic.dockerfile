ARG BASE_IMAGE=ros:noetic-ros-base
FROM $BASE_IMAGE

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
