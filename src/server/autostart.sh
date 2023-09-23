#!/bin/bash

# setup ros environment
source "/opt/ros/$ROS_DISTRO/setup.bash"

cd /opt/server/src/ && npm install && nodemon index.js
