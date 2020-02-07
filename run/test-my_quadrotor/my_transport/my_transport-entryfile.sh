#!/bin/bash
set -e

# setup ros2 environment with my packages included
source /opt/my_ros_ws/install/setup.bash
exec "$@"
