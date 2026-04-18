#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SEED="${1:-123}"
HOVER_ALTITUDE="${2:-1.0}"
ROSCORE_STARTED=0

cleanup() {
  local exit_code=$?
  trap - EXIT INT TERM

  if ! stop_rmua_runtime "${ROSCORE_STARTED}"; then
    if [[ "${exit_code}" -eq 0 ]]; then
      exit_code=1
    fi
  fi

  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

ensure_runtime_dirs
ensure_simulator_exists
ensure_workspace_built
source_workspace

stop_rmua_runtime 0 || true

if ! ros_master_is_up; then
  stop_background_process roscore
  start_roscore_process roscore
  ROSCORE_STARTED=1
  wait_for_ros_master 20
fi

start_simulator_process simulator "${SEED}" offscreen

echo "[rmua] 使用种子 ${SEED} 启动后台模式模拟器与 PWM 飞控栈"
echo "[rmua] 飞控默认待机，稳定后可执行 ./scripts/enable_pwm_controller.sh 进入悬停控制"

run_managed_roslaunch_stack rmua_flight_control pwm_hover_stack.launch hover_altitude_m:="${HOVER_ALTITUDE}"
