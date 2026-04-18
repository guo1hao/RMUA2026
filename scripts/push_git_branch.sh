#!/usr/bin/env bash

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

usage() {
  cat <<'EOF'
用法:
  ./scripts/push_git_branch.sh <main|master> [commit_message] [remote_name]

示例:
  ./scripts/push_git_branch.sh main
  ./scripts/push_git_branch.sh master "fix: update pending2 transition"
  ./scripts/push_git_branch.sh main "chore: sync local changes" origin

说明:
  1. 第一个参数必须是 main 或 master。
  2. 第二个参数可选，作为 commit message；不传则自动生成一条带时间戳的消息。
  3. 第三个参数可选，默认 remote 为 origin。
  4. 脚本会执行 git add -A，并在存在未提交改动时自动 commit，然后推送当前 HEAD 到目标分支。
EOF
}

TARGET_BRANCH="${1:-}"
COMMIT_MESSAGE="${2:-}"
REMOTE_NAME="${3:-origin}"

if [[ -z "${TARGET_BRANCH}" ]]; then
  usage
  exit 1
fi

if [[ "${TARGET_BRANCH}" != "main" && "${TARGET_BRANCH}" != "master" ]]; then
  echo "[git] 目标分支只能是 main 或 master，当前输入: ${TARGET_BRANCH}" >&2
  usage
  exit 1
fi

if ! git -C "${WORKSPACE_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "[git] 当前目录不是 git 仓库: ${WORKSPACE_ROOT}" >&2
  exit 1
fi

if ! git -C "${WORKSPACE_ROOT}" remote get-url "${REMOTE_NAME}" >/dev/null 2>&1; then
  echo "[git] remote 不存在: ${REMOTE_NAME}" >&2
  exit 1
fi

CURRENT_BRANCH="$(git -C "${WORKSPACE_ROOT}" branch --show-current)"
REMOTE_URL="$(git -C "${WORKSPACE_ROOT}" remote get-url "${REMOTE_NAME}")"
TIMESTAMP="$(date '+%Y-%m-%d %H:%M:%S')"

if [[ -z "${COMMIT_MESSAGE}" ]]; then
  if [[ -n "${CURRENT_BRANCH}" ]]; then
    COMMIT_MESSAGE="chore: push ${CURRENT_BRANCH} to ${TARGET_BRANCH} (${TIMESTAMP})"
  else
    COMMIT_MESSAGE="chore: push detached HEAD to ${TARGET_BRANCH} (${TIMESTAMP})"
  fi
fi

echo "[git] workspace: ${WORKSPACE_ROOT}"
echo "[git] remote: ${REMOTE_NAME} -> ${REMOTE_URL}"
echo "[git] current branch: ${CURRENT_BRANCH:-detached HEAD}"
echo "[git] target branch: ${TARGET_BRANCH}"

git -C "${WORKSPACE_ROOT}" add -A

if [[ -n "$(git -C "${WORKSPACE_ROOT}" status --short)" ]]; then
  echo "[git] 检测到未提交改动，执行 commit"
  git -C "${WORKSPACE_ROOT}" commit -m "${COMMIT_MESSAGE}"
else
  echo "[git] 没有新的未提交改动，跳过 commit"
fi

echo "[git] 开始推送 HEAD -> ${REMOTE_NAME}/${TARGET_BRANCH}"
git -C "${WORKSPACE_ROOT}" push -u "${REMOTE_NAME}" "HEAD:${TARGET_BRANCH}"

echo "[git] 推送完成"