#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

POSE_TOPIC="${1:-/rmua/sensors/drone_1/pose_gt}"
IMU_TOPIC="${2:-/rmua/sensors/drone_1/imu}"
ENABLE_TOPIC="${3:-/rmua/control/basic/enable}"
WAIT_TIMEOUT="${4:-20}"

wait_for_topic_data() {
  local topic="$1"
  local timeout_seconds="$2"
  local elapsed=0

  while (( elapsed < timeout_seconds )); do
    if timeout 3s rostopic echo -n 1 "${topic}" >/dev/null 2>&1; then
      return 0
    fi
    elapsed=$((elapsed + 1))
  done

  echo "[rmua] 等待 topic 数据超时: ${topic}" >&2
  return 1
}

source_workspace

wait_for_topic_data "${POSE_TOPIC}" "${WAIT_TIMEOUT}"
wait_for_topic_data "${IMU_TOPIC}" "${WAIT_TIMEOUT}"

echo "[rmua] 传感器已就绪，启用 PWM 控制器"
rostopic pub -1 "${ENABLE_TOPIC}" std_msgs/Bool "data: true"
