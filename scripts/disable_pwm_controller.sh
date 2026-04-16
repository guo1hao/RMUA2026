#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
source "${SCRIPT_DIR}/common.sh"

ENABLE_TOPIC="${1:-/rmua/control/basic/enable}"

source_workspace

echo "[rmua] 关闭 PWM 控制器"
rostopic pub -1 "${ENABLE_TOPIC}" std_msgs/Bool "data: false"
