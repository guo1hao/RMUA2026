#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
SIM_ROOT="${RMUA_SIM_ROOT:-/home/hao/robomaster/RMUA/2026/simulator_12.0.0.5}"
RUNTIME_ROOT="${WORKSPACE_ROOT}/.runtime"
PID_ROOT="${RUNTIME_ROOT}/pids"
LOG_ROOT="${RUNTIME_ROOT}/logs"
ACTIVE_STACK_PID=""
ACTIVE_MONITOR_PID=""
SELF_PGID="$(ps -o pgid= -p $$ 2>/dev/null | tr -d ' ' || true)"

shell_quote_join() {
  local quoted_parts=()
  local value=""
  local quoted_value=""

  for value in "$@"; do
    printf -v quoted_value '%q' "${value}"
    quoted_parts+=("${quoted_value}")
  done

  local IFS=' '
  echo "${quoted_parts[*]}"
}

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

get_process_group_id() {
  local pid="$1"

  ps -o pgid= -p "${pid}" 2>/dev/null | tr -d ' ' || true
}

signal_process_group_or_pid() {
  local pid="$1"
  local signal_name="$2"
  local process_group_id=""

  process_group_id="$(get_process_group_id "${pid}")"
  if [[ -n "${process_group_id}" && "${process_group_id}" != "${SELF_PGID}" ]]; then
    kill "-${signal_name}" -- "-${process_group_id}" >/dev/null 2>&1 || true
    return 0
  fi

  kill "-${signal_name}" "${pid}" >/dev/null 2>&1 || true
}

wait_for_process_exit() {
  local pid="$1"
  local retries="${2:-20}"
  local interval_seconds="${3:-0.2}"

  for _ in $(seq 1 "${retries}"); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return 0
    fi
    sleep "${interval_seconds}"
  done

  return 1
}

process_is_zombie() {
  local pid="$1"
  local process_state=""

  process_state="$(ps -o stat= -p "${pid}" 2>/dev/null | tr -d ' ' || true)"
  [[ "${process_state}" == Z* ]]
}

wait_for_process_inactive() {
  local pid="$1"
  local retries="${2:-20}"
  local interval_seconds="${3:-0.2}"

  for _ in $(seq 1 "${retries}"); do
    if ! kill -0 "${pid}" >/dev/null 2>&1; then
      return 0
    fi

    if process_is_zombie "${pid}"; then
      return 0
    fi

    sleep "${interval_seconds}"
  done

  return 1
}

stop_background_monitor_process() {
  local pid="$1"

  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    return 0
  fi

  pkill -TERM -P "${pid}" >/dev/null 2>&1 || true
  kill -TERM "${pid}" >/dev/null 2>&1 || true
  if ! wait_for_process_inactive "${pid}" 10 0.1; then
    pkill -KILL -P "${pid}" >/dev/null 2>&1 || true
    kill -KILL "${pid}" >/dev/null 2>&1 || true
  fi

  if ! wait_for_process_inactive "${pid}" 10 0.1; then
    echo "[rmua] 警告：后台监控进程未能正常退出，PID=${pid}" >&2
    return 1
  fi

  wait "${pid}" >/dev/null 2>&1 || true
  return 0
}

stop_managed_process() {
  local pid="$1"
  local name="$2"
  local initial_signal="${3:-TERM}"

  if ! kill -0 "${pid}" >/dev/null 2>&1; then
    return 0
  fi

  signal_process_group_or_pid "${pid}" "${initial_signal}"
  if ! wait_for_process_inactive "${pid}" 20 0.2; then
    signal_process_group_or_pid "${pid}" TERM
  fi
  if ! wait_for_process_inactive "${pid}" 20 0.2; then
    signal_process_group_or_pid "${pid}" KILL
  fi

  if ! wait_for_process_inactive "${pid}" 20 0.2; then
    echo "[rmua] 警告：${name} 未能在超时内退出，PID=${pid}" >&2
    return 1
  fi

  wait "${pid}" >/dev/null 2>&1 || true
  echo "[rmua] 已停止 ${name}，PID=${pid}"
}

stop_rmua_runtime() {
  local stop_roscore="${1:-0}"
  local stop_status=0

  stop_managed_stack_processes || stop_status=1
  stop_background_process sensor_stack || stop_status=1
  stop_background_process pwm_stack || stop_status=1
  stop_background_process mission_stack || stop_status=1
  stop_background_process official_mission_stack || stop_status=1
  stop_stale_rmua_ros_processes || stop_status=1
  stop_background_process simulator || stop_status=1

  if [[ "${stop_roscore}" == "1" ]]; then
    stop_background_process roscore || stop_status=1
    stop_stale_ros_master_processes || stop_status=1
  fi

  stop_stale_simulator_processes || stop_status=1
  check_renderer_processes || stop_status=1

  return "${stop_status}"
}

restart_local_ros_master() {
  stop_background_process roscore
  stop_stale_ros_master_processes
  start_roscore_process roscore
  wait_for_ros_master 20
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

start_roscore_process() {
  local name="$1"

  start_background_process "${name}" roscore
}

get_background_process_pid() {
  local name="$1"
  local pid_file="${PID_ROOT}/${name}.pid"

  if [[ ! -f "${pid_file}" ]]; then
    return 1
  fi

  cat "${pid_file}"
}

start_process_exit_monitor() {
  local watched_pid="$1"
  local target_pid="$2"
  local watched_name="${3:-process}"

  (
    trap 'exit 0' INT TERM

    while kill -0 "${watched_pid}" >/dev/null 2>&1; do
      sleep 0.5 &
      wait "$!" || exit 0
    done

    echo "[rmua] ${watched_name} 进程已退出，开始释放当前运行栈"
    signal_process_group_or_pid "${target_pid}" INT
    if wait_for_process_exit "${target_pid}" 20 0.2; then
      exit 0
    fi
    signal_process_group_or_pid "${target_pid}" TERM
  ) &

  echo "$!"
}

stop_managed_stack_processes() {
  if [[ -n "${ACTIVE_MONITOR_PID}" ]]; then
    stop_background_monitor_process "${ACTIVE_MONITOR_PID}" || true
    ACTIVE_MONITOR_PID=""
  fi

  if [[ -n "${ACTIVE_STACK_PID}" ]]; then
    stop_managed_process "${ACTIVE_STACK_PID}" "当前 ROS launch 栈" INT
    ACTIVE_STACK_PID=""
  fi
}

run_managed_roslaunch_stack() {
  local launch_package="$1"
  local launch_file="$2"
  shift 2

  local simulator_pid=""
  simulator_pid="$(get_background_process_pid simulator || true)"

  local roslaunch_command=""
  roslaunch_command="$(shell_quote_join roslaunch "${launch_package}" "${launch_file}" "$@")"

  bash -lc "set -euo pipefail; source /opt/ros/noetic/setup.bash; if [[ -f \"${WORKSPACE_ROOT}/devel/setup.bash\" ]]; then source \"${WORKSPACE_ROOT}/devel/setup.bash\"; fi; exec ${roslaunch_command}" &
  ACTIVE_STACK_PID=$!
  local stack_pid="${ACTIVE_STACK_PID}"
  local launch_exit_code=0
  local interrupted=0
  local previous_int_trap=""
  local previous_term_trap=""

  previous_int_trap="$(trap -p INT || true)"
  previous_term_trap="$(trap -p TERM || true)"

  handle_managed_stack_signal() {
    interrupted=1
    stop_managed_stack_processes
  }

  trap 'handle_managed_stack_signal' INT TERM

  if [[ -n "${simulator_pid}" ]]; then
    ACTIVE_MONITOR_PID="$(start_process_exit_monitor "${simulator_pid}" "${stack_pid}" simulator)"
  fi

  wait "${stack_pid}" || launch_exit_code=$?
  trap - INT TERM
  if [[ -n "${previous_int_trap}" ]]; then
    eval "${previous_int_trap}"
  fi
  if [[ -n "${previous_term_trap}" ]]; then
    eval "${previous_term_trap}"
  fi
  stop_managed_stack_processes

  if [[ "${interrupted}" == "1" && "${launch_exit_code}" -eq 0 ]]; then
    launch_exit_code=130
  fi

  return "${launch_exit_code}"
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

stop_stale_ros_master_processes() {
  mapfile -t stale_roscore_pids < <(pgrep -x roscore || true)
  mapfile -t stale_rosmaster_pids < <(pgrep -x rosmaster || true)

  local all_pids=()
  if [[ "${#stale_roscore_pids[@]}" -gt 0 ]]; then
    all_pids+=("${stale_roscore_pids[@]}")
  fi
  if [[ "${#stale_rosmaster_pids[@]}" -gt 0 ]]; then
    all_pids+=("${stale_rosmaster_pids[@]}")
  fi

  if [[ "${#all_pids[@]}" -eq 0 ]]; then
    return 0
  fi

  local pid=""
  local remaining_pids=()
  declare -A seen_pids=()
  for pid in "${all_pids[@]}"; do
    if [[ -n "${pid}" && -z "${seen_pids[${pid}]:-}" ]]; then
      seen_pids[${pid}]=1
      remaining_pids+=("${pid}")
    fi
  done

  for pid in "${remaining_pids[@]}"; do
    signal_process_group_or_pid "${pid}" TERM
  done
  for _ in $(seq 1 20); do
    local still_running=()
    for pid in "${remaining_pids[@]}"; do
      if kill -0 "${pid}" >/dev/null 2>&1; then
        still_running+=("${pid}")
      fi
    done

    if [[ "${#still_running[@]}" -eq 0 ]]; then
      break
    fi

    remaining_pids=("${still_running[@]}")
    sleep 0.2
  done

  if [[ "${#remaining_pids[@]}" -gt 0 ]]; then
    for pid in "${remaining_pids[@]}"; do
      signal_process_group_or_pid "${pid}" KILL
    done
  fi

  echo "[rmua] 已清理遗留 roscore/rosmaster 进程: ${remaining_pids[*]:-${all_pids[*]}}"
}

stop_background_process() {
  local name="$1"
  local pid_file="${PID_ROOT}/${name}.pid"
  local stop_status=0

  if [[ ! -f "${pid_file}" ]]; then
    return 0
  fi

  local pid
  pid="$(cat "${pid_file}")"

  if kill -0 "${pid}" >/dev/null 2>&1; then
    stop_managed_process "${pid}" "${name}" TERM || stop_status=$?
  fi

  rm -f "${pid_file}"

  if [[ "${name}" == "simulator" ]]; then
    stop_stale_simulator_processes || stop_status=$?
  fi

  return "${stop_status}"
}
