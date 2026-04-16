#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${RMUA_SIM_ROOT:-/home/hao/robomaster/RMUA/2026/simulator_12.0.0.5}"
RUNTIME_ROOT="${WORKSPACE_ROOT}/.runtime"
PID_ROOT="${RUNTIME_ROOT}/pids"
LOG_ROOT="${RUNTIME_ROOT}/logs"

generate_random_seed() {
  local raw_seed
  raw_seed="$(od -An -N4 -tu4 /dev/urandom | tr -d ' ')"

  if [[ -z "${raw_seed}" ]]; then
    raw_seed="$((RANDOM + 1))"
  fi

  echo "$((raw_seed % 1000000 + 1))"
}

ensure_runtime_dirs() {
  mkdir -p "${PID_ROOT}" "${LOG_ROOT}"
}

stop_rmua_runtime() {
  local stop_roscore="${1:-0}"

  stop_stale_rmua_ros_processes
  stop_background_process simulator

  if [[ "${stop_roscore}" == "1" ]]; then
    stop_background_process roscore
  fi

  stop_stale_simulator_processes
  check_renderer_processes
}

source_workspace() {
  source /opt/ros/noetic/setup.bash
  if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
    source "${WORKSPACE_ROOT}/devel/setup.bash"
  fi
}

ensure_workspace_built() {
  if [[ -f "${WORKSPACE_ROOT}/devel/setup.bash" ]]; then
    return 0
  fi

  echo "[rmua] devel/setup.bash 不存在，先执行 catkin_make"
  pushd "${WORKSPACE_ROOT}" >/dev/null
  source /opt/ros/noetic/setup.bash
  catkin_make
  popd >/dev/null
}

start_roslaunch_stack() {
  local launch_package="$1"
  local launch_file="$2"
  shift 2

  source_workspace
  roslaunch "${launch_package}" "${launch_file}" "$@"
}

ensure_simulator_exists() {
  if [[ ! -d "${SIM_ROOT}" ]]; then
    echo "[rmua] 模拟器目录不存在: ${SIM_ROOT}" >&2
    exit 1
  fi

  if [[ ! -x "${SIM_ROOT}/Build/LinuxNoEditor/RMUA.sh" ]]; then
    echo "[rmua] 模拟器可执行文件不存在: ${SIM_ROOT}/Build/LinuxNoEditor/RMUA.sh" >&2
    exit 1
  fi
}

start_simulator_process() {
  local name="$1"
  local seed="$2"
  local mode="${3:-render}"
  local render_flag=""

  if [[ "${mode}" == "offscreen" ]]; then
    render_flag=" -RenderOffscreen"
  fi

  start_background_process \
    "${name}" \
    bash -lc "source /opt/ros/noetic/setup.bash && cd \"${SIM_ROOT}\" && exec ./Build/LinuxNoEditor/RMUA.sh seed \"${seed}\"${render_flag}"
}

ros_master_is_up() {
  rostopic list >/dev/null 2>&1
}

wait_for_ros_master() {
  local timeout_seconds="${1:-20}"
  local elapsed=0

  while ! ros_master_is_up; do
    if (( elapsed >= timeout_seconds * 2 )); then
      echo "[rmua] 等待 roscore 超时" >&2
      return 1
    fi
    sleep 0.5
    elapsed=$((elapsed + 1))
  done
}

start_background_process() {
  local name="$1"
  shift

  local pid_file="${PID_ROOT}/${name}.pid"
  local log_file="${LOG_ROOT}/${name}.log"

  setsid "$@" >"${log_file}" 2>&1 &
  local pid=$!
  echo "${pid}" >"${pid_file}"
  echo "[rmua] 已启动 ${name}，PID=${pid}，日志=${log_file}"
}

stop_stale_simulator_processes() {
  local simulator_launcher="${SIM_ROOT}/Build/LinuxNoEditor/RMUA.sh"
  local simulator_binary="${SIM_ROOT}/Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping"
  mapfile -t launcher_pids < <(pgrep -f "${simulator_launcher}" || true)
  mapfile -t binary_pids < <(pgrep -f "${simulator_binary}" || true)

  local stale_pids=()
  local pid
  declare -A seen_pids=()
  for pid in "${launcher_pids[@]}" "${binary_pids[@]}"; do
    if [[ -n "${pid}" && -z "${seen_pids[${pid}]:-}" ]]; then
      seen_pids[${pid}]=1
      stale_pids+=("${pid}")
    fi
  done

  if [[ "${#stale_pids[@]}" -eq 0 ]]; then
    return 0
  fi

  kill "${stale_pids[@]}" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    local remaining=()
    for pid in "${stale_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        remaining+=("${pid}")
      fi
    done

    if [[ "${#remaining[@]}" -eq 0 ]]; then
      break
    fi

    stale_pids=("${remaining[@]}")
    sleep 0.2
  done

  if [[ "${#stale_pids[@]}" -gt 0 ]]; then
    kill -9 "${stale_pids[@]}" >/dev/null 2>&1 || true
  fi

  echo "[rmua] 已清理遗留 simulator 进程: ${stale_pids[*]}"
}

check_renderer_processes() {
  local simulator_launcher="${SIM_ROOT}/Build/LinuxNoEditor/RMUA.sh"
  local simulator_binary="${SIM_ROOT}/Build/LinuxNoEditor/RMUA/Binaries/Linux/RMUA-Linux-Shipping"
  mapfile -t launcher_lines < <(pgrep -af "${simulator_launcher}" || true)
  mapfile -t binary_lines < <(pgrep -af "${simulator_binary}" || true)

  local remaining=()
  if [[ "${#launcher_lines[@]}" -gt 0 ]]; then
    remaining+=("${launcher_lines[@]}")
  fi
  if [[ "${#binary_lines[@]}" -gt 0 ]]; then
    remaining+=("${binary_lines[@]}")
  fi

  if [[ "${#remaining[@]}" -eq 0 ]]; then
    echo "[rmua] 未发现残留渲染/模拟器进程"
    return 0
  fi

  echo "[rmua] 仍存在渲染/模拟器进程:"
  printf '%s\n' "${remaining[@]}"
  return 1
}

stop_stale_rmua_ros_processes() {
  mapfile -t stale_pids < <(pgrep -f "${WORKSPACE_ROOT}/devel/lib/rmua_" || true)
  mapfile -t stale_launch_pids < <(pgrep -f "roslaunch .*rmua_" || true)

  local all_pids=()
  if [[ "${#stale_pids[@]}" -gt 0 ]]; then
    all_pids+=("${stale_pids[@]}")
  fi
  if [[ "${#stale_launch_pids[@]}" -gt 0 ]]; then
    all_pids+=("${stale_launch_pids[@]}")
  fi

  if [[ "${#all_pids[@]}" -eq 0 ]]; then
    return 0
  fi

  kill "${all_pids[@]}" >/dev/null 2>&1 || true
  for _ in $(seq 1 20); do
    local remaining=()
    for pid in "${all_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        remaining+=("${pid}")
      fi
    done

    if [[ "${#remaining[@]}" -eq 0 ]]; then
      break
    fi

    all_pids=("${remaining[@]}")
    sleep 0.2
  done

  if [[ "${#all_pids[@]}" -gt 0 ]]; then
    kill -9 "${all_pids[@]}" >/dev/null 2>&1 || true
  fi

  echo "[rmua] 已清理遗留 ROS 进程: ${all_pids[*]}"
}

stop_background_process() {
  local name="$1"
  local pid_file="${PID_ROOT}/${name}.pid"

  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi

  local pid
  pid="$(cat "${pid_file}")"

  if kill -0 "${pid}" >/dev/null 2>&1; then
    kill -TERM -- "-${pid}" >/dev/null 2>&1 || kill "${pid}" >/dev/null 2>&1 || true
    for _ in $(seq 1 20); do
      if ! kill -0 "${pid}" >/dev/null 2>&1; then
        break
      fi
      sleep 0.2
    done
    if kill -0 "${pid}" >/dev/null 2>&1; then
      kill -KILL -- "-${pid}" >/dev/null 2>&1 || kill -9 "${pid}" >/dev/null 2>&1 || true
    fi
    echo "[rmua] 已停止 ${name}，PID=${pid}"
  fi

  rm -f "${pid_file}"

  if [[ "${name}" == "simulator" ]]; then
    stop_stale_simulator_processes
  fi
}
