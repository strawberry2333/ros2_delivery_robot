#!/usr/bin/env bash
# smoke_test.sh — 端到端冒烟测试：提交订单 → 自动确认 → 验证完成
# 用法: bash scripts/smoke_test.sh
# 超时: 420 秒 (wall-clock)
set -euo pipefail

TIMEOUT=420
POLL_INTERVAL=2
DEMO_PID=""
CLEANUP_DONE=false

# DeliveryStatus.msg 状态枚举
readonly STATE_WAITING_LOAD=2
readonly STATE_WAITING_UNLOAD=4
readonly STATE_COMPLETE=5
readonly STATE_FAILED=6

# ROS2 服务/话题名称
readonly SRV_SUBMIT_ORDER="/submit_order"
readonly SRV_CONFIRM_LOAD="/confirm_load"
readonly SRV_CONFIRM_UNLOAD="/confirm_unload"
readonly TOPIC_DELIVERY_STATUS="/delivery_status"

# 清理本项目的 ROS2/Gazebo 残留进程
# 使用 delivery_bringup 特征字符串限定范围，避免误杀不相关的 ROS2 会话
kill_stale_processes() {
  echo "[smoke] 清理残留进程..."
  # 先杀 launch 父进程（会级联终止其子进程组）
  pkill -f "ros2 launch delivery_bringup" 2>/dev/null || true
  # 杀本项目特有的节点（名称足够唯一，不会误杀其他 ROS2 会话）
  pkill -f "delivery_manager" 2>/dev/null || true
  pkill -f "delivery_executor" 2>/dev/null || true
  pkill -f "delivery_lifecycle_manager" 2>/dev/null || true
  # Gazebo 仿真进程（server 用 warehouse 限定，GUI client 用 -g 匹配）
  pkill -f "ruby.*gz.*warehouse" 2>/dev/null || true
  pkill -f "gz sim.*warehouse" 2>/dev/null || true
  pkill -f "gz sim -g" 2>/dev/null || true
}

cleanup() {
  if $CLEANUP_DONE; then return; fi
  CLEANUP_DONE=true
  echo "[smoke] 清理中..."
  # 终止后台状态订阅（整个进程组，包括 ros2 topic echo 和 grep）
  if [[ -n "${STATUS_SUB_PID:-}" ]] && kill -0 "$STATUS_SUB_PID" 2>/dev/null; then
    kill -- -"$STATUS_SUB_PID" 2>/dev/null || true
  fi
  rm -f "${STATUS_FILE:-}" 2>/dev/null || true
  # 终止 demo launch 进程树
  if [[ -n "$DEMO_PID" ]] && kill -0 "$DEMO_PID" 2>/dev/null; then
    kill -- -"$DEMO_PID" 2>/dev/null || true
    wait "$DEMO_PID" 2>/dev/null || true
  fi
  kill_stale_processes
  echo "[smoke] 清理完成"
}
trap cleanup EXIT INT TERM

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../ros2_ws" && pwd)"

# 环境准备
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
export TURTLEBOT3_MODEL=waffle_pi

# 启动前先清理旧进程，避免 /clock 多 publisher 污染
kill_stale_processes
sleep 2

echo "[smoke] 启动 demo (headless, rviz:=false)..."
setsid ros2 launch delivery_bringup demo.launch.py rviz:=false &
DEMO_PID=$!

# 等待 /submit_order 服务可用
echo "[smoke] 等待 /submit_order 服务..."
SECONDS=0
while ! ros2 service list 2>/dev/null | grep -q "$SRV_SUBMIT_ORDER"; do
  if (( SECONDS > 120 )); then
    echo "[smoke] FAIL: 等待 $SRV_SUBMIT_ORDER 超时 (120s)"
    exit 1
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] $SRV_SUBMIT_ORDER 可用 (${SECONDS}s)"

# 额外等待 /delivery_status topic 有 publisher，确认 executor 已激活
echo "[smoke] 等待 delivery_executor 激活..."
SECONDS=0
while ! ros2 topic info "$TOPIC_DELIVERY_STATUS" 2>/dev/null | grep -q "Publisher count: [1-9]"; do
  if (( SECONDS > 60 )); then
    echo "[smoke] WARN: 等待 /delivery_status publisher 超时，继续执行"
    break
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] executor 就绪 (${SECONDS}s)"

# 提交订单（带重试：system_ready_ 可能在服务注册后才置 true）
echo "[smoke] 提交订单 station_A -> station_C..."
SECONDS=0
while (( SECONDS < 60 )); do
  SUBMIT_OUTPUT=$(ros2 service call "$SRV_SUBMIT_ORDER" delivery_interfaces/srv/SubmitOrder \
    "{order: {order_id: 'smoke_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}" \
    2>&1 || true)
  if echo "$SUBMIT_OUTPUT" | grep -q "accepted=True\|accepted: True"; then
    echo "$SUBMIT_OUTPUT" | head -5
    echo "[smoke] 订单已接受"
    break
  fi
  echo "[smoke] 订单未被接受，${SECONDS}s 后重试... ($(echo "$SUBMIT_OUTPUT" | grep -o 'reason:.*' | head -1))"
  sleep "$POLL_INTERVAL"
done
if ! echo "$SUBMIT_OUTPUT" | grep -q "accepted=True\|accepted: True"; then
  echo "[smoke] FAIL: 提交订单超时 (60s)"
  echo "$SUBMIT_OUTPUT"
  exit 1
fi

# 启动持久订阅，将最新状态写入临时文件（避免 --once 丢失一次性消息）
# PYTHONUNBUFFERED=1 确保 Python CLI（ros2 topic echo）stdout 不缓冲
# 用 setsid 将整条 pipeline 放入独立进程组，清理时一次性终止
STATUS_FILE=$(mktemp /tmp/smoke_status.XXXXXX)
setsid bash -c 'PYTHONUNBUFFERED=1 ros2 topic echo "'"$TOPIC_DELIVERY_STATUS"'" --no-arr 2>/dev/null \
  | stdbuf -oL grep -E "^state:" \
  > "'"$STATUS_FILE"'"' &
STATUS_SUB_PID=$!

# 轮询 /delivery_status，在关键阶段自动确认
echo "[smoke] 轮询配送状态..."
LOAD_CONFIRMED=false
UNLOAD_CONFIRMED=false
SECONDS=0

while (( SECONDS < TIMEOUT )); do
  # 读取持久订阅捕获的最新状态
  STATUS_LINE=$(tail -1 "$STATUS_FILE" 2>/dev/null || true)
  STATE=$(echo "$STATUS_LINE" | awk '/^state:/{print $2}')

  case "$STATE" in
    "$STATE_WAITING_LOAD")
      if ! $LOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_LOAD, 发送 confirm_load..."
        ros2 service call "$SRV_CONFIRM_LOAD" std_srvs/srv/Trigger 2>&1 | head -3
        LOAD_CONFIRMED=true
      fi
      ;;
    "$STATE_WAITING_UNLOAD")
      if ! $UNLOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_UNLOAD, 发送 confirm_unload..."
        ros2 service call "$SRV_CONFIRM_UNLOAD" std_srvs/srv/Trigger 2>&1 | head -3
        UNLOAD_CONFIRMED=true
      fi
      ;;
    "$STATE_COMPLETE")
      echo "[smoke] PASS: 配送完成 (state=$STATE_COMPLETE), 耗时 ${SECONDS}s"
      exit 0
      ;;
    "$STATE_FAILED")
      echo "[smoke] FAIL: 配送失败 (state=$STATE_FAILED)"
      exit 1
      ;;
    *)
      if [[ -n "$STATE" ]]; then
        echo "[smoke] 未知状态: state=$STATE (原始: $STATUS_LINE)"
      fi
      ;;
  esac

  sleep "$POLL_INTERVAL"
done

echo "[smoke] FAIL: 超时 (${TIMEOUT}s)"
exit 1
