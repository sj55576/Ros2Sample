#!/usr/bin/env bash
set -euo pipefail

if ! command -v colcon >/dev/null 2>&1; then
  echo "error: colcon is not installed. Install python3-colcon-common-extensions." >&2
  exit 127
fi

packages=$(colcon list --names-only 2>/dev/null || true)
if [[ -z "${packages}" ]]; then
  echo "No ROS 2 packages found; skipping colcon lint/test discovery."
  exit 0
fi

colcon test \
  --event-handlers console_direct+ \
  --packages-select ${packages} \
  --ctest-args -R "(lint|copyright|flake8|pep257|xmllint|cppcheck|cpplint)" "$@"
