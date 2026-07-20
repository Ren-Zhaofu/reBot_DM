#!/usr/bin/env bash

set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 service list 2>/dev/null | grep -Fxq /rebotarm/gravity_compensation/stop; then
  echo "重力补偿服务未启动。" >&2
  exit 1
fi
exec ros2 service call /rebotarm/gravity_compensation/stop std_srvs/srv/Trigger "{}"
