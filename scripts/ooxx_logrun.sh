#!/usr/bin/env bash
# 运行命令并将终端输出同时保存到 ooxx/log/ 目录。
# 用法:
#   rosrun ooxx ooxx_logrun.sh roslaunch ooxx chassis_search.launch
#   rosrun ooxx ooxx_logrun.sh p1-test roslaunch ooxx chassis_search.launch
set -euo pipefail

usage() {
  cat <<'EOF'
用法:
  ooxx_logrun.sh <命令...>
  ooxx_logrun.sh <标签> <命令...>

示例:
  ooxx_logrun.sh roslaunch ooxx chassis_search.launch
  ooxx_logrun.sh p1-test roslaunch ooxx chassis_search.launch

日志保存到 ooxx/log/H-M.txt 或 ooxx/log/H-M-标签.txt
可通过环境变量 OOXX_LOG_DIR 指定日志目录。
EOF
}

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

label=""
if [ $# -ge 2 ] && [[ "$1" != roslaunch && "$1" != rosrun && "$1" != roscore && "$1" != *.* && "$1" != /* ]]; then
  label="$1"
  shift
fi

if [ $# -lt 1 ]; then
  usage
  exit 1
fi

find_log_dir() {
  if [ -n "${OOXX_LOG_DIR:-}" ]; then
    mkdir -p "$OOXX_LOG_DIR"
    echo "$OOXX_LOG_DIR"
    return
  fi

  local pkg candidates candidate
  pkg="$(rospack find ooxx 2>/dev/null || true)"
  candidates=()
  if [ -n "$pkg" ]; then
    candidates+=("$pkg/log" "$(dirname "$pkg")/log")
    if [[ "$pkg" == *"/share/ooxx" ]]; then
      candidates+=("$(dirname "$(dirname "$(dirname "$pkg")")")/src/ooxx/log")
    fi
  fi
  candidates+=("$HOME/experiment_ws/src/ooxx/log")

  for candidate in "${candidates[@]}"; do
    if [ -d "$candidate" ] || [ -d "$(dirname "$candidate")" ]; then
      mkdir -p "$candidate"
      echo "$candidate"
      return
    fi
  done

  mkdir -p "./log"
  echo "$(cd ./log && pwd)"
}

LOG_DIR="$(find_log_dir)"
stamp="$(date +%-H-%-M)"
if [ -n "$label" ]; then
  base="${stamp}-${label}"
else
  base="${stamp}"
fi
LOG_FILE="${LOG_DIR}/${base}.txt"
if [ -e "$LOG_FILE" ]; then
  base="${stamp}-$(date +%-S)"
  if [ -n "$label" ]; then
    base="${stamp}-${label}-$(date +%-S)"
  fi
  LOG_FILE="${LOG_DIR}/${base}.txt"
fi

cmd_str="$*"
started_at="$(date '+%Y-%m-%d %H:%M:%S')"
host_name="$(hostname 2>/dev/null || echo unknown)"
cwd="$(pwd)"

{
  echo "# cmd: ${cmd_str}"
  echo "# start: ${started_at}"
  echo "# host: ${host_name}"
  echo "# cwd: ${cwd}"
  echo "# log: ${LOG_FILE}"
  if [ -n "$label" ]; then
    echo "# label: ${label}"
  fi
  echo "---"
} > "$LOG_FILE"

echo "[ooxx_logrun] 日志 -> ${LOG_FILE}"
echo "[ooxx_logrun] 命令: ${cmd_str}"
echo "---"

set +e
"$@" 2>&1 | tee -a "$LOG_FILE"
exit_code=${PIPESTATUS[0]}
set -e

{
  echo "---"
  echo "# end: $(date '+%Y-%m-%d %H:%M:%S')"
  echo "# exit: ${exit_code}"
} >> "$LOG_FILE"

echo "[ooxx_logrun] 结束 exit=${exit_code}，已保存 ${LOG_FILE}"
exit "$exit_code"
