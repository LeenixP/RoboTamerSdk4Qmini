#!/bin/bash

# ============================================================
# Qmini 机器人启动脚本 - 带日志轮转管理
# ============================================================

set -e

# 配置参数
LOG_DIR="/home/lckfb/robot-logs"
MAX_LOG_SIZE_MB=50          # 单个日志文件最大大小 (MB)
MAX_LOG_FILES=50            # 保留的最大日志文件数
LOG_ROTATE_INTERVAL=3600    # 日志轮转检查间隔 (秒)

# 创建日志目录
mkdir -p "$LOG_DIR"

# 日志轮转函数
rotate_log_if_needed() {
    local log_file="$1"
    
    if [ ! -f "$log_file" ]; then
        return 0
    fi
    
    local size_mb=$(du -m "$log_file" 2>/dev/null | cut -f1)
    
    if [ "$size_mb" -ge "$MAX_LOG_SIZE_MB" ]; then
        local timestamp=$(date +%Y%m%d_%H%M%S)
        local rotated_file="${log_file%.log}_${timestamp}.log"
        mv "$log_file" "$rotated_file"
        gzip "$rotated_file" 2>/dev/null || true
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Log rotated: $rotated_file.gz"
    fi
}

# 清理旧日志函数
cleanup_old_logs() {
    local file_count=$(find "$LOG_DIR" -name "*.log" -o -name "*.log.gz" 2>/dev/null | wc -l)
    
    if [ "$file_count" -gt "$MAX_LOG_FILES" ]; then
        local delete_count=$((file_count - MAX_LOG_FILES))
        find "$LOG_DIR" -name "*.log" -o -name "*.log.gz" 2>/dev/null | \
            sort | head -n "$delete_count" | xargs rm -f 2>/dev/null || true
        echo "[$(date '+%Y-%m-%d %H:%M:%S')] Cleaned up $delete_count old log files"
    fi
}

# 后台日志轮转守护进程
start_log_daemon() {
    local log_file="$1"
    local pid_file="$LOG_DIR/.log_daemon.pid"
    
    # 如果已有守护进程在运行，先停止
    if [ -f "$pid_file" ]; then
        local old_pid=$(cat "$pid_file")
        if kill -0 "$old_pid" 2>/dev/null; then
            kill "$old_pid" 2>/dev/null || true
        fi
        rm -f "$pid_file"
    fi
    
    # 启动新的守护进程
    (
        while true; do
            sleep "$LOG_ROTATE_INTERVAL"
            rotate_log_if_needed "$log_file"
            cleanup_old_logs
        done
    ) &
    echo $! > "$pid_file"
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Log daemon started (PID: $(cat $pid_file))"
}

# 等待 PS4 手柄
echo "[$(date '+%Y-%m-%d %H:%M:%S')] Waiting for PS4 controller..."
while ! grep -ql "Wireless Controller" /sys/class/input/event*/device/name 2>/dev/null; do
    echo "[$(date '+%Y-%m-%d %H:%M:%S')] Controller not found, retrying in 5s..."
    sleep 5
done
echo "[$(date '+%Y-%m-%d %H:%M:%S')] PS4 controller connected!"

# 生成日志文件名
LOG_TIME=$(date +%Y_%m_%d-%H)
LOG_FILE="$LOG_DIR/run_interface_$(date +%Y%m%d_%H%M%S).log"

cd /home/lckfb/RoboTamerSdk4Qmini/bin/

# 打印启动信息
echo "============================================================"
echo "Qmini Robot Control System"
echo "Start Time: $(date '+%Y-%m-%d %H:%M:%S')"
echo "Log File: $LOG_FILE"
echo "Max Log Size: ${MAX_LOG_SIZE_MB}MB"
echo "Max Log Files: $MAX_LOG_FILES"
echo "============================================================"

# 启动日志轮转守护进程
start_log_daemon "$LOG_FILE"

# 清理旧日志
cleanup_old_logs

# 运行主程序
# 使用 stdbuf 禁用缓冲，确保日志实时写入
exec stdbuf -oL -eL /home/lckfb/RoboTamerSdk4Qmini/bin/run_interface 2>&1 | tee "$LOG_FILE"