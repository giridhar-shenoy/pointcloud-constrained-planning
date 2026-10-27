#!/bin/bash
# Source ROS 2 Lyrical (and the workspace overlay, if built) before running the command.
set -e
# shellcheck disable=SC1091
source /opt/ros/lyrical/setup.bash
if [ -f /ws/install/setup.bash ]; then
  # shellcheck disable=SC1091
  source /ws/install/setup.bash
fi
exec "$@"
