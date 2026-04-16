#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SEED="${1:-123}"
TARGET_INDEX="${2:-69}"
PATH_CSV="${3:-${WORKSPACE_ROOT}/drone_path.csv}"
LOOKAHEAD_DISTANCE="${4:-30.0}"
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

start_simulator_process simulator "${SEED}" offscreen
SIMULATOR_STARTED=1

echo "[rmua] 使用种子 ${SEED} 启动后台模式模拟器与路径任务"
echo "[rmua] 目标路径文件: ${PATH_CSV}"
echo "[rmua] 目标 waypoint index: ${TARGET_INDEX}"

start_roslaunch_stack \
  rmua_flight_control \
  pwm_path_mission.launch \
  path_csv:="${PATH_CSV}" \
  target_waypoint_index:="${TARGET_INDEX}" \
  lookahead_distance_m:="${LOOKAHEAD_DISTANCE}"