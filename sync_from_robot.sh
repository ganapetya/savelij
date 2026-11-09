#!/usr/bin/env bash
# Copy the Jetson workspace src into this checkout, then push to GitHub.
# One direction only: robot -> this folder -> origin. This script never
# fetches, pulls, or rebases. If origin has commits this checkout does
# not have, the push stops and leaves the remote as it is.
set -euo pipefail

ROBOT_HOST="${YAHBOOM_HOST:-jetson@192.168.0.120}"
ROBOT_SRC="${YAHBOOM_SRC:-/home/jetson/yahboomcar_ros2_ws/yahboomcar_ws/src/}"
BRANCH="${YAHBOOM_BRANCH:-master}"

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$ROOT"

if [[ ! -d .git ]]; then
  echo "error: $ROOT is not a git checkout" >&2
  exit 1
fi

current="$(git rev-parse --abbrev-ref HEAD)"
if [[ "$current" != "$BRANCH" ]]; then
  echo "error: checkout is on '$current', expected '$BRANCH'" >&2
  exit 1
fi

echo "rsync ${ROBOT_HOST}:${ROBOT_SRC} -> ${ROOT}/src/"
# build/, install/, and log/ sit inside the robot src tree from colcon.
# They are generated and are not part of the source we publish.
rsync -az --delete \
  --exclude 'build/' \
  --exclude 'install/' \
  --exclude 'log/' \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  --exclude '.git/' \
  "${ROBOT_HOST}:${ROBOT_SRC}" \
  "${ROOT}/src/"

git add -A

if git diff --cached --quiet; then
  echo "No changes. Nothing to push."
  exit 0
fi

if [[ $# -gt 0 ]]; then
  msg="$*"
else
  msg="sync from yahboom $(date -Iseconds)"
fi

git commit -m "$msg"
git push origin "$BRANCH"
echo "Pushed to origin/${BRANCH}."
