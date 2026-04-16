#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

SEED="${1:-$(generate_random_seed)}"
TARGET_INDEX="${2:--1}"
PATH_CSV="${3:-${WORKSPACE_ROOT}/drone_path.csv}"
LOOKAHEAD_DISTANCE="${4:-16.0}"
ROSCORE_STARTED=0
SIMULATOR_STARTED=0

cleanup() {
  local exit_code=$?
  local stop_roscore=0

  if [[ "${ROSCORE_STARTED}" -eq 1 ]]; then
    stop_roscore=1
  fi

  if ! stop_rmua_runtime "${stop_roscore}"; then
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
  start_background_process roscore roscore
  ROSCORE_STARTED=1
  wait_for_ros_master 20
fi

start_simulator_process simulator "${SEED}" render
SIMULATOR_STARTED=1

echo "[rmua] 使用种子 ${SEED} 启动可视模式模拟器与路径任务"
echo "[rmua] 目标路径文件: ${PATH_CSV}"
if [[ "${TARGET_INDEX}" == "-1" ]]; then
  echo "[rmua] 目标 waypoint index: 末尾 waypoint（完整路径）"
else
  echo "[rmua] 目标 waypoint index: ${TARGET_INDEX}"
fi
echo "[rmua] 当前路径任务即比赛模式控制栈"

start_roslaunch_stack \
  rmua_flight_control \
  pwm_path_mission.launch \
  path_csv:="${PATH_CSV}" \
  target_waypoint_index:="${TARGET_INDEX}" \
  lookahead_distance_m:="${LOOKAHEAD_DISTANCE}"