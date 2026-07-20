#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_CHANNEL="${1:-can0}"
if [[ $# -gt 0 ]]; then
  shift
fi
if [[ ! "${CAN_CHANNEL}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
  echo "非法 CAN 接口名称: ${CAN_CHANNEL}" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
set -u

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "首次运行：正在构建 ${WORKSPACE}"
  "${WORKSPACE}/scripts/build.sh"
fi

set +u
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 pkg prefix rebotarm_bringup >/dev/null 2>&1; then
  echo "当前 install 中没有 rebotarm_bringup，正在重新构建。" >&2
  "${WORKSPACE}/scripts/build.sh"
  set +u
  source "${WORKSPACE}/install/setup.bash"
  set -u
fi

"${WORKSPACE}/scripts/scan_motors.sh" "${CAN_CHANNEL}"

echo
echo "扫描通过，启动真实 DM 驱动。请保持本终端运行。"
echo "另开终端执行: ${WORKSPACE}/scripts/safe_home.sh"
exec ros2 launch rebotarm_bringup driver.launch.py \
  use_real_hardware:=true \
  channel:="${CAN_CHANNEL}" \
  publish_rate:=20.0 \
  use_rviz:=false \
  allow_safe_home:=true \
  "$@"
