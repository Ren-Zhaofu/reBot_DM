#!/usr/bin/env bash

set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  set +u
  source /opt/ros/humble/setup.bash
  set -u
fi

if ! command -v colcon >/dev/null 2>&1; then
  echo "缺少 colcon，请先安装 ROS 2 Humble 和 python3-colcon-common-extensions。" >&2
  exit 1
fi

cd "${WORKSPACE}"
colcon build --symlink-install "$@"
