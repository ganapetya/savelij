#!/usr/bin/env bash
# Launch rqt_graph with basic display diagnostics.
# Expects host X11: DISPLAY, /tmp/.X11-unix mounted, xhost +local:root on host.
#
# Note: do NOT use `set -u` before sourcing ROS setup.bash — ament scripts
# reference optional vars (e.g. AMENT_TRACE_SETUP_FILES) that may be unset.

echo "=== rqt_graph start ==="
echo "DISPLAY=${DISPLAY:-<unset>}"
echo "QT_QPA_PLATFORM=${QT_QPA_PLATFORM:-<unset>}"
echo "ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-<unset>}"
echo "RMW_IMPLEMENTATION=${RMW_IMPLEMENTATION:-<unset>}"
echo "CYCLONEDDS_URI=${CYCLONEDDS_URI:-<unset>}"
echo "ROS_LOCALHOST_ONLY=${ROS_LOCALHOST_ONLY:-<unset>}"
echo

echo "=== /tmp/.X11-unix ==="
if ls -la /tmp/.X11-unix 2>/dev/null; then
  :
else
  echo "WARNING: /tmp/.X11-unix missing or empty — mount it and set DISPLAY on the host."
fi
echo

if [ -z "${DISPLAY:-}" ]; then
  echo "ERROR: DISPLAY is not set. On host: export DISPLAY=:1 && xhost +local:root"
  exit 1
fi

# Optional ament debug vars — avoid unbound-variable issues if someone re-enables set -u
export AMENT_TRACE_SETUP_FILES="${AMENT_TRACE_SETUP_FILES:-}"
export AMENT_PYTHON_EXECUTABLE="${AMENT_PYTHON_EXECUTABLE:-}"
export COLCON_TRACE="${COLCON_TRACE:-}"
export AMENT_PREFIX_PATH="${AMENT_PREFIX_PATH:-}"
export CMAKE_PREFIX_PATH="${CMAKE_PREFIX_PATH:-}"

# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash

echo "=== ros2 node list (DDS check) ==="
if ! ros2 node list; then
  echo "WARNING: ros2 node list failed (DDS/config). Still starting rqt_graph…"
fi
echo

echo "=== starting rqt_graph ==="
exec ros2 run rqt_graph rqt_graph "$@"

