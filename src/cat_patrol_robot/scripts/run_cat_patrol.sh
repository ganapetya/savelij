#!/bin/bash
# Source workspace overlay and start cat patrol stack (use from systemd timer).
set -euo pipefail
WS="${CAT_PATROL_WS:-$HOME/yahboomcar_ros2_ws/yahboomcar_ws}"
ROBOT="${CAT_PATROL_ROBOT:-x3}"
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash
else
  echo "ROS 2 Humble setup.bash not found" >&2
  exit 1
fi
# shellcheck source=/dev/null
source "$WS/install/setup.bash"
exec ros2 launch cat_patrol_robot cat_patrol.launch.py "robot:=$ROBOT" "$@"
