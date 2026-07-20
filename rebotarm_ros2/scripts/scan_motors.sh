#!/usr/bin/env bash

set -euo pipefail

CAN_CHANNEL="${1:-can0}"
if [[ ! "${CAN_CHANNEL}" =~ ^[a-zA-Z0-9_.-]+$ ]]; then
  echo "非法 CAN 接口名称: ${CAN_CHANNEL}" >&2
  exit 2
fi
if ! command -v motorbridge-cli >/dev/null 2>&1; then
  echo "缺少 motorbridge-cli，无法扫描电机。" >&2
  exit 1
fi
if [[ ! -d "/sys/class/net/${CAN_CHANNEL}" ]]; then
  echo "CAN 接口不存在: ${CAN_CHANNEL}" >&2
  exit 1
fi
CAN_FLAGS="$(<"/sys/class/net/${CAN_CHANNEL}/flags")"
if (( (CAN_FLAGS & 1) == 0 )); then
  echo "CAN 接口未启动: ${CAN_CHANNEL}" >&2
  echo "请先配置 1 Mbps 并执行: sudo ip link set ${CAN_CHANNEL} up" >&2
  exit 1
fi

exec motorbridge-cli scan \
  --vendor damiao \
  --transport socketcan \
  --channel "${CAN_CHANNEL}" \
  --model 4340P \
  --start-id 1 \
  --end-id 7 \
  --timeout-ms 500
