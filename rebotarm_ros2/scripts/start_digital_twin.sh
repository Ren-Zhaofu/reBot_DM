#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_CHANNEL="${1:-can0}"
if [[ ! "${CAN_CHANNEL}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
  echo "非法 CAN 接口名称: ${CAN_CHANNEL}" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

"${WORKSPACE}/scripts/scan_motors.sh" "${CAN_CHANNEL}"
echo
echo "启动真实反馈数字孪生：RViz 将同步显示六轴机械臂。"
echo "运动、回零和标零均保持锁定；在本终端按 Ctrl+C 可完整退出。"
exec ros2 launch rebotarm_bringup driver.launch.py \
  use_real_hardware:=true \
  channel:="${CAN_CHANNEL}" \
  publish_rate:=20.0 \
  use_rviz:=true \
  allow_enable:=false \
  allow_safe_home:=false \
  allow_joint_motion:=false \
  allow_calibration:=false
