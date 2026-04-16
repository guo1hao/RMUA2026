#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SEED="${1:-123}"
ROSCORE_STARTED=0
SIMULATOR_STARTED=0

cleanup() {
  local exit_code=$?
  if [[ "${SIMULATOR_STARTED}" -eq 1 ]]; then
    stop_background_process simulator
  fi
  if [[ "${ROSCORE_STARTED}" -eq 1 ]]; then
    stop_background_process roscore
  fi
  exit "${exit_code}"
}

trap cleanup EXIT INT TERM

ensure_runtime_dirs
ensure_simulator_exists
ensure_workspace_built
source_workspace

stop_stale_rmua_ros_processes
stop_background_process simulator

if ! ros_master_is_up; then
  stop_background_process roscore
  start_background_process roscore roscore
  ROSCORE_STARTED=1
  wait_for_ros_master 20
fi

start_simulator_process simulator "${SEED}" render
SIMULATOR_STARTED=1

echo "[rmua] 使用种子 ${SEED} 启动渲染模式模拟器"
echo "[rmua] 现在启动 ROS 工程"

roslaunch rmua_sensor_hub competition_stack.launch
