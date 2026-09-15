#!/bin/bash

echo "当前用户: $(whoami)"
echo "SUDO_USER: $SUDO_USER"
echo "HOME: $HOME"
echo "PWD: $(pwd)"

# 获取原始用户的家目录
if [ -n "$SUDO_USER" ]; then
    USER_HOME=$(getent passwd "$SUDO_USER" | cut -d: -f6)
else
    USER_HOME="$HOME"
fi

echo "USER_HOME: $USER_HOME"

# build 目录
BUILD_DIR="$USER_HOME/workspace/noetix-m1-yaocao/build"
echo "程序目录: $BUILD_DIR"

# 检查目录是否存在
if [ ! -d "$BUILD_DIR" ]; then
    echo "错误: 找不到目录: $BUILD_DIR"
    exit 1
fi

# 列出目录内容
ls -la "$BUILD_DIR"

# 选择可执行文件（兼容两种名字）
CANDIDATES=(
  "$BUILD_DIR/noetix-m1-yaocao"
  "$BUILD_DIR/noetix-yaocao"
)

PROGRAM_PATH=""
for f in "${CANDIDATES[@]}"; do
    if [ -f "$f" ] && [ -x "$f" ]; then
        PROGRAM_PATH="$f"
        break
    fi
done

echo "最终执行文件: $PROGRAM_PATH"

if [ -z "$PROGRAM_PATH" ]; then
    echo "错误: 未找到可执行文件（候选: ${CANDIDATES[*]}）"
    exit 1
fi

# -----------------------------
# 执行程序并自动重启失败
# -----------------------------
MAX_RETRIES=1
RETRY_DELAY=1
retry_count=0

while [ $retry_count -lt $MAX_RETRIES ]; do
    echo "正在执行程序 (尝试 $((retry_count+1))/$MAX_RETRIES)..."
    
    "$PROGRAM_PATH"
    EXIT_CODE=$?

    if [ $EXIT_CODE -eq 0 ]; then
        echo "程序正常退出"
        exit 0
    else
        echo "程序异常退出，错误码: $EXIT_CODE"
        retry_count=$((retry_count+1))
        echo "等待 $RETRY_DELAY 秒后重试..."
        sleep $RETRY_DELAY
    fi
done

echo "错误: 程序连续 $MAX_RETRIES 次启动失败，退出"
exit 1