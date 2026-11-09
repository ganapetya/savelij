#!/usr/bin/env bash
# =============================================================================
# sync_robot_to_host.sh
# =============================================================================
# Runs on the HOST machine (NOT on the Jetson).
#
# What it does, in order:
#   1. rsync the entire workspace from the Jetson to /home/bots/all/
#      (one-way mirror, the canonical "what's on the robot right now").
#   2. rsync cat_patrol_robot/   from the mirror to /opt/robots/cat_patrol_robot/
#   3. rsync odom_drift_checker/ from the mirror to /opt/robots/odom_drift_checker/
#   4. Copy the two modified Yahboom files into
#      /opt/robots/cat_patrol_robot/yahboom_overlay/  so they get tracked
#      with cat_patrol_robot's git repo (without polluting the upstream
#      Yahboom checkout on the robot).
#   5. rsync ALL OTHER packages from yahboomcar_ws/src/ (the vendor + community
#      packages we depend on) to /opt/robots/yahboom_ws_peter/src/.  Skips the
#      packages already tracked individually (cat_patrol_robot,
#      odom_drift_checker) and all build artefacts.
#
# After it finishes, you do the normal git workflow in each /opt/robots
# project:
#     cd /opt/robots/cat_patrol_robot   && git add . && git commit && git push
#     cd /opt/robots/odom_drift_checker && git add . && git commit && git push
#     cd /opt/robots/yahboom_ws_peter   && git add . && git commit && git push
#
# This is a PUSH-ONLY workflow: the robot is the source of truth, and we
# never pull git changes back into the robot. Edits go robot → host → git.
# =============================================================================

set -uo pipefail
# NOTE: We deliberately do NOT use `set -e`. rsync legitimately returns
# nonzero on partial transfers (codes 23 / 24) when there are e.g. nested
# build dirs that don't sync cleanly. We want the script to keep going
# past those benign warnings and run steps 2 and 3 anyway.

ROBOT_HOST="${ROBOT_HOST:-jetson@192.168.0.120}"
ROBOT_WS="${ROBOT_WS:-/home/jetson/yahboomcar_ros2_ws}"
HOST_MIRROR="${HOST_MIRROR:-/home/bots/all}"
SHARED_BASE="${SHARED_BASE:-/opt/robots}"
YAHBOOM_WS_PETER="${YAHBOOM_WS_PETER:-${SHARED_BASE}/yahboom_ws_peter}"

# Packages to mirror to per-package shared dirs (one git repo each).
# Add new ones here as you create them.
TRACKED_PACKAGES=(
    cat_patrol_robot
    odom_drift_checker
)

# Vendor files to copy into cat_patrol_robot/yahboom_overlay/.
# Format: "src_relative_to_yahboomcar_ws/src    dst_relative_to_yahboom_overlay"
YAHBOOM_OVERLAY_FILES=(
    "yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py    yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py"
    "yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py    yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py"
)

echo "===================================================================="
echo "STEP 1/4: rsync from robot to host mirror"
echo "  ${ROBOT_HOST}:${ROBOT_WS}/  ->  ${HOST_MIRROR}/"
echo "===================================================================="
# Exclude colcon's build artefacts and ros logs -- they're not source-of-truth
# and the build/ dir occasionally contains symlink loops that confuse rsync.
rsync -avz --progress \
    --exclude='build/' --exclude='install/' --exclude='log/' \
    --exclude='__pycache__/' --exclude='*.pyc' \
    --exclude='.ros_inspect_logs/' \
    "${ROBOT_HOST}:${ROBOT_WS}/" "${HOST_MIRROR}/" \
    || echo "[warn] rsync from robot exited nonzero (likely benign partial-transfer); continuing"

WS_SRC="${HOST_MIRROR}/yahboomcar_ws/src"

echo
echo "===================================================================="
echo "STEP 2/4: rsync each tracked package to its shared dir"
echo "===================================================================="
for pkg in "${TRACKED_PACKAGES[@]}"; do
    src="${WS_SRC}/${pkg}/"
    dst="${SHARED_BASE}/${pkg}/"
    if [ ! -d "${src}" ]; then
        echo "  SKIP ${pkg}: ${src} not found in mirror"
        continue
    fi
    mkdir -p "${dst}"
    echo "  ${src}  ->  ${dst}"
    rsync -a --delete --exclude='.git' --exclude='build/' --exclude='install/' --exclude='log/' \
          --exclude='__pycache__/' --exclude='*.pyc' --exclude='.ros_inspect_logs/' \
          "${src}" "${dst}"
done

echo
echo "===================================================================="
echo "STEP 3/4: copy Yahboom-modified files into cat_patrol_robot/yahboom_overlay/"
echo "===================================================================="
OVERLAY_BASE="${SHARED_BASE}/cat_patrol_robot/yahboom_overlay"
for line in "${YAHBOOM_OVERLAY_FILES[@]}"; do
    # Split on whitespace.
    src_rel="$(echo "${line}" | awk '{print $1}')"
    dst_rel="$(echo "${line}" | awk '{print $2}')"
    src="${WS_SRC}/${src_rel}"
    dst="${OVERLAY_BASE}/${dst_rel}"
    if [ ! -f "${src}" ]; then
        echo "  SKIP: ${src} not found"
        continue
    fi
    mkdir -p "$(dirname "${dst}")"
    cp "${src}" "${dst}"
    echo "  ${src_rel}  ->  yahboom_overlay/${dst_rel}"
done

# Drop a tiny README into yahboom_overlay so future-you knows what it is.
cat > "${OVERLAY_BASE}/README.md" << 'EOF'
# yahboom_overlay

Files in this directory are **modifications to the Yahboom-shipped
`yahboomcar_bringup` package**. We keep them here (rather than committing
inside the Yahboom checkout) so a fresh clone of Yahboom's repo doesn't
clobber our changes.

To re-apply after re-cloning Yahboom:

    cd ~/yahboomcar_ros2_ws/yahboomcar_ws/src
    cp -v cat_patrol_robot/yahboom_overlay/yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py \
          yahboomcar_bringup/yahboomcar_bringup/Mcnamu_driver_X3.py
    cp -v cat_patrol_robot/yahboom_overlay/yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py \
          yahboomcar_bringup/launch/yahboomcar_bringup_X3_launch.py
    colcon build --symlink-install --packages-select yahboomcar_bringup

The diffs vs upstream are described in plan.md (Phase 0 findings) and
phase0-status.md.
EOF

echo
echo "===================================================================="
echo "STEP 4/4: rsync all OTHER packages to ${YAHBOOM_WS_PETER}/src/"
echo "===================================================================="
WS_PETER_SRC="${YAHBOOM_WS_PETER}/src"
mkdir -p "${WS_PETER_SRC}"

# Build the rsync exclude list from TRACKED_PACKAGES so the per-package
# repos in /opt/robots/<pkg>/ remain the single source of truth for those.
EXCLUDES=()
for pkg in "${TRACKED_PACKAGES[@]}"; do
    EXCLUDES+=( "--exclude=/${pkg}/" )
done

# Mirror everything else under yahboomcar_ws/src/ (vendor and community
# packages: yahboomcar_bringup, yahboomcar_base_node, yahboomcar_msgs, etc.).
# --delete keeps yahboom_ws_peter exactly in sync with what's on the robot,
# so packages removed on the robot are also removed on the host repo.
rsync -a --delete \
    "${EXCLUDES[@]}" \
    --exclude='.git' \
    --exclude='build/' \
    --exclude='install/' \
    --exclude='log/' \
    --exclude='__pycache__/' \
    --exclude='*.pyc' \
    --exclude='.ipynb_checkpoints/' \
    --exclude='.ros_inspect_logs/' \
    "${WS_SRC}/" "${WS_PETER_SRC}/"
echo "  ${WS_SRC}/  ->  ${WS_PETER_SRC}/  (excluding: ${TRACKED_PACKAGES[*]})"

# Drop a small README + .gitignore at the workspace root the first time
# the dir is created. These files live OUTSIDE the rsync target (src/)
# so --delete doesn't remove them.
if [ ! -f "${YAHBOOM_WS_PETER}/README.md" ]; then
    cat > "${YAHBOOM_WS_PETER}/README.md" << 'EOF'
# yahboom_ws_peter

Mirror of the **vendor + community** ROS 2 packages from the cat-patrol
robot's workspace, *excluding* the two custom packages we maintain
separately (`cat_patrol_robot/` and `odom_drift_checker/`).

This repo exists so that a fresh checkout of the project is reproducible:
clone this repo, drop it into `your_workspace/src/`, then clone
`cat_patrol_robot/` and `odom_drift_checker/` alongside, and `colcon
build` should produce the same workspace as the live robot.

## Layout

    yahboom_ws_peter/
    └── src/
        ├── yahboomcar_bringup/        ← Yahboom vendor (with our
        │                                 trim-parameter modifications;
        │                                 see cat_patrol_robot/yahboom_overlay
        │                                 for the patch record)
        ├── yahboomcar_base_node/      ← Yahboom vendor (C++ odometry integrator)
        ├── yahboomcar_description/    ← Yahboom vendor (URDFs and meshes)
        ├── yahboomcar_msgs/           ← Yahboom vendor (message types)
        └── ...                        ← all other yahboomcar_* packages
                                          plus laserscan_to_point_pulisher,
                                          robot_pose_publisher_ros2, etc.

## Sync workflow

This repo is populated by running, on the host machine (not the robot):

    /opt/robots/cat_patrol_robot/scripts/sync_robot_to_host.sh

That script rsyncs the live workspace from the Jetson to the host mirror,
then mirrors each package to the right place in `/opt/robots/`.

Push-only: do NOT git-pull into this repo to bring changes back to the
robot. Edits flow robot → host → git, never the other way.
EOF
    echo "  Wrote ${YAHBOOM_WS_PETER}/README.md"
fi

if [ ! -f "${YAHBOOM_WS_PETER}/.gitignore" ]; then
    cat > "${YAHBOOM_WS_PETER}/.gitignore" << 'EOF'
build/
install/
log/
__pycache__/
*.pyc
.ipynb_checkpoints/
.vscode/
.idea/
*.swp
EOF
    echo "  Wrote ${YAHBOOM_WS_PETER}/.gitignore"
fi

echo
echo "===================================================================="
echo "Done. Next steps (manual):"
echo "===================================================================="
for pkg in "${TRACKED_PACKAGES[@]}"; do
    echo "  cd ${SHARED_BASE}/${pkg} && git status"
done
echo "  cd ${YAHBOOM_WS_PETER} && git status"
