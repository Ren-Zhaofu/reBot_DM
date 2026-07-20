#!/usr/bin/env bash

set -euo pipefail

WORKSPACE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
ASSUME_YES=false
if [[ "${1:-}" == "--yes" ]]; then
  ASSUME_YES=true
elif [[ $# -gt 0 ]]; then
  echo "用法: $0 [--yes]" >&2
  exit 2
fi

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 /opt/ros/humble/setup.bash" >&2
  exit 1
fi
if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  echo "工作区尚未构建，请先运行: ${WORKSPACE}/scripts/start_real.sh" >&2
  exit 1
fi

set +u
source /opt/ros/humble/setup.bash
source "${WORKSPACE}/install/setup.bash"
set -u

echo "等待 /rebotarm/safe_home 服务（最多 10 秒）..."
service_ready=false
for _ in {1..20}; do
  if ros2 service list 2>/dev/null | grep -Fxq /rebotarm/safe_home; then
    service_ready=true
    break
  fi
  sleep 0.5
done
if [[ "${service_ready}" != true ]]; then
  echo "服务未启动。请在另一个终端先运行:" >&2
  echo "  ${WORKSPACE}/scripts/start_real.sh" >&2
  exit 1
fi

if [[ "${ASSUME_YES}" != true ]]; then
  if [[ ! -t 0 ]]; then
    echo "非交互运行必须显式传入 --yes。" >&2
    exit 2
  fi
  echo "警告：机械臂将运动到已标定的关节零位。"
  read -r -p "确认周围无人、无障碍物并已准备急停？输入 yes 继续: " answer
  if [[ "${answer}" != "yes" ]]; then
    echo "已取消。"
    exit 2
  fi
fi

exec ros2 service call /rebotarm/safe_home std_srvs/srv/Trigger "{}"
