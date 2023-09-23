#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd /nodejs_ros_server/ && npm install && nodemon index.js
