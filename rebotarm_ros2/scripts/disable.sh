#!/usr/bin/env bash

set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 service list 2>/dev/null | grep -Fxq /rebotarm/disable; then
  echo "失能服务未启动。" >&2
  exit 1
fi
echo "警告：电机失能后，无抱闸机械臂可能立即下垂。"
exec ros2 service call /rebotarm/disable std_srvs/srv/Trigger "{}"
