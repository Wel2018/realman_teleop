#!/bin/bash

########################################
# 遥操作服务
# weikang
# 2026-01-22
########################################


LOGFILE="/home/sy/wk/log/run_teleop_server.log"
mkdir -p "$(dirname "$LOGFILE")" 

{
  echo "========== $(date) =========="
  echo "激活 conda 环境..."
  source /home/sy/miniconda3/bin/activate teleop
  cd /home/sy/wk/projects || exit 1
  
  echo "[$(date)] 启动..."
  python -m realman_teleop_http_server
  EXIT_CODE=$?
  echo "[$(date)] 退出，状态码: $EXIT_CODE"
} # >> "$LOGFILE" 2>&1
