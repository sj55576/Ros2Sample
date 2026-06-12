#!/usr/bin/env bash
set -euo pipefail

rosdistro="${1:-${ROS_DISTRO:-jazzy}}"

if ! command -v rosdep >/dev/null 2>&1; then
  echo "error: rosdep is not installed. Install python3-rosdep." >&2
  exit 127
fi

if [[ ! -d src ]]; then
  mkdir -p src
fi

rosdep install \
  --from-paths src \
  --ignore-src \
  --rosdistro "${rosdistro}" \
  -r \
  -y
