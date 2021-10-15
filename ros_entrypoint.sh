#!/bin/bash
set -e

# setup ros1 environment
source "/opt/ros/$ROS_DISTRO/setup.bash"
source "/ros1_ws/devel/setup.bash"

exec "$@"