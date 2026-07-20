#!/usr/bin/env bash

set -euo pipefail
WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

if [[ "${1:-}" == "--yes" ]]; then
  shift
elif [[ -t 0 ]]; then
  echo "警告：重力补偿会使能电机并进入低刚度 MIT 力矩控制。"
  echo "请托住机械臂，确保周围无人、无障碍物且急停可用。"
  read -r -p "输入 yes 继续: " answer
  [[ "${answer}" == "yes" ]] || { echo "已取消。"; exit 2; }
else
  echo "非交互执行必须传入 --yes" >&2
  exit 2
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

if ! ros2 service list 2>/dev/null | grep -Fxq /rebotarm/gravity_compensation/start; then
  echo "重力补偿服务未启动。请先在另一终端运行：" >&2
  echo "  ${WORKSPACE}/scripts/start_real.sh can0 allow_gravity_compensation:=true" >&2
  exit 1
fi
echo "停止补偿: ${WORKSPACE}/scripts/stop_gravity_compensation.sh"
echo "彻底失能: ${WORKSPACE}/scripts/disable.sh"
exec ros2 service call /rebotarm/gravity_compensation/start std_srvs/srv/Trigger "{}"
