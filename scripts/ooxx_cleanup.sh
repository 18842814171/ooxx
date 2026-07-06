#!/usr/bin/env bash
# 清理残留 ooxx / 底盘 / 雷达相关进程（分步启动前可选执行）
set -euo pipefail
echo "[ooxx_cleanup] 停止残留节点..."
rosnode kill -a 2>/dev/null || true
pkill -f "ooxx_node.py" 2>/dev/null || true
pkill -f "robot_controller_node" 2>/dev/null || true
pkill -f "rplidar" 2>/dev/null || true
echo "[ooxx_cleanup] 完成"
