#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
CAN_CHANNEL="${1:-can0}"
if [[ ! "${CAN_CHANNEL}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
  echo "非法 CAN 接口名称: ${CAN_CHANNEL}" >&2
  exit 2
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble 环境。" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "工作区尚未构建，请先运行 ${WORKSPACE}/scripts/build.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 pkg prefix rebotarm_moveit_config >/dev/null 2>&1; then
  echo "install 中缺少 MoveIt 配置，请重新运行 scripts/build.sh" >&2
  exit 1
fi
"${WORKSPACE}/scripts/scan_motors.sh" "${CAN_CHANNEL}"

echo
echo "扫描通过，启动 MoveIt 与真实 DM 驱动。"
echo "运动和安全回零接口已解锁，但不会自动发送运动命令。"
echo "另开终端执行: ${WORKSPACE}/scripts/test_moveit_real.sh"
exec ros2 launch rebotarm_moveit_config moveit.launch.py \
  use_real_hardware:=true \
  channel:="${CAN_CHANNEL}" \
  allow_safe_home:=true \
  allow_joint_motion:=true
