#!/bin/bash

# Source ROS distro environment and local catkin workspace
source "/opt/ros/$ROS_DISTRO/setup.bash" && source "$CATKIN_WS/devel/setup.bash"

cd /root/catkin_ws

exec "$@"