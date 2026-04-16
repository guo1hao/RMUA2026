#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

COMMAND="${1:-start}"

if [[ "${COMMAND}" == "stop" ]]; then
	echo "[rmua] 停止当前比赛模式寻点控制栈"
	stop_rmua_runtime 1
	exit 0
fi

if [[ "${COMMAND}" == "restart" ]]; then
	echo "[rmua] 重启当前比赛模式寻点控制栈"
	stop_rmua_runtime 1 || true
	shift
	exec "${SCRIPT_DIR}/start_path_mission.sh" "$@"
fi

if [[ "${COMMAND}" == "start" ]]; then
	shift || true
fi

echo "[rmua] 启动当前比赛模式寻点控制栈"
exec "${SCRIPT_DIR}/start_path_mission.sh" "$@"