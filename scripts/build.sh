#!/usr/bin/env bash
set -euo pipefail

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not installed. Install python3-colcon-common-extensions." >&2
  exit 127
fi

if [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  set +u
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
  set -u
fi

colcon build --symlink-install "$@"
