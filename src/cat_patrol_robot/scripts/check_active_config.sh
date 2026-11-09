#!/bin/bash
# =============================================================================
# check_active_config.sh
# -----------------------------------------------------------------------------
# Verifies which cat_patrol_robot config will actually be used when you
# launch with `ros2 launch cat_patrol_robot cat_patrol.launch.py`.
#
# Prevents the "I edited src yaml but install yaml is still old" trap.
#
# Usage:
#   bash check_active_config.sh
# =============================================================================
set -eo pipefail

WS="${CAT_PATROL_WS:-$HOME/yahboomcar_ros2_ws/yahboomcar_ws}"
SRC_YAML="$WS/src/cat_patrol_robot/config/cat_patrol_params.yaml"
INSTALL_YAML="$WS/install/cat_patrol_robot/share/cat_patrol_robot/config/cat_patrol_params.yaml"

# ROS setup.bash uses unset vars internally; do not enable nounset here.
if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck source=/dev/null
  source /opt/ros/humble/setup.bash || true
fi
if [[ -f "$WS/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "$WS/install/setup.bash" || true
fi

echo "=========== CAT PATROL CONFIG CHECK ==========="
echo "  source YAML : $SRC_YAML"
echo "  install YAML: $INSTALL_YAML"
echo

if command -v ros2 >/dev/null 2>&1; then
  echo "ros2 pkg prefix cat_patrol_robot:"
  ros2 pkg prefix cat_patrol_robot || true
  echo
fi

python3 - "$SRC_YAML" "$INSTALL_YAML" <<'PY'
import sys, hashlib
try:
    import yaml
except Exception as e:
    print("PyYAML not installed:", e); sys.exit(0)

KEYS = [
    "odom_angular_scale",
    "capture_completion_extra_deg",
    "capture_yaw_tolerance",
    "capture_frame_count",
    "capture_turn_speed",
    "capture_kp",
    "capture_min_angular_speed",
    "angular_speed",
    "linear_speed",
    "depth_obstacle_min_m",
    "depth_max_age_sec",
    "voltage_topic",
    "low_voltage_warn_v",
]

def load(path):
    try:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        return data.get("/**", {}).get("ros__parameters", {})
    except Exception as ex:
        return {"<error>": str(ex)}

src = load(sys.argv[1])
ins = load(sys.argv[2])

print(f"{'key':32s} {'src':>16s} {'install':>16s}  diff?")
for k in KEYS:
    s = src.get(k, "<missing>")
    i = ins.get(k, "<missing>")
    diff = "" if s == i else "  <-- MISMATCH"
    print(f"{k:32s} {str(s):>16s} {str(i):>16s}{diff}")

def sha(path):
    try:
        h = hashlib.sha256()
        with open(path, "rb") as f:
            h.update(f.read())
        return h.hexdigest()[:12]
    except Exception:
        return "n/a"

print()
print("file sha256 (first 12):")
print("  src    :", sha(sys.argv[1]))
print("  install:", sha(sys.argv[2]))
PY

echo
echo "If you see MISMATCH above, rebuild and re-source:"
echo "  cd $WS && colcon build --packages-select cat_patrol_robot && source install/setup.bash"
echo "==============================================="
