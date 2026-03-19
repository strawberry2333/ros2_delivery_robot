#!/usr/bin/env bash
# smoke_test.sh — 端到端冒烟测试：提交订单 → 自动确认 → 验证完成
# 用法: bash scripts/smoke_test.sh
# 超时: 420 秒 (wall-clock)
set -euo pipefail

TIMEOUT=420
POLL_INTERVAL=2
DEMO_PID=""
CLEANUP_DONE=false

# 清理所有 ROS2/Gazebo 相关残留进程
kill_stale_processes() {
  echo "[smoke] 清理残留进程..."
  pkill -f "ros2 launch delivery_bringup" 2>/dev/null || true
  pkill -f "delivery_manager" 2>/dev/null || true
  pkill -f "delivery_executor" 2>/dev/null || true
  pkill -f "delivery_lifecycle_manager" 2>/dev/null || true
  pkill -f "bt_navigator" 2>/dev/null || true
  pkill -f "ros_gz_bridge" 2>/dev/null || true
  pkill -f "ros_gz_image" 2>/dev/null || true
  pkill -f "gzserver" 2>/dev/null || true
  pkill -f "gzclient" 2>/dev/null || true
  pkill -f "ruby.*gz" 2>/dev/null || true
  pkill -f "rviz2" 2>/dev/null || true
  # 等待进程退出
  sleep 2
}

cleanup() {
  if $CLEANUP_DONE; then return; fi
  CLEANUP_DONE=true
  echo "[smoke] 清理中..."
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
ros2 daemon stop 2>/dev/null || true
ros2 daemon start 2>/dev/null || true

echo "[smoke] 启动 demo (headless, rviz:=false)..."
setsid ros2 launch delivery_bringup demo.launch.py rviz:=false &
DEMO_PID=$!

# 等待 /submit_order 服务可用
echo "[smoke] 等待 /submit_order 服务..."
SECONDS=0
while ! ros2 service list 2>/dev/null | grep -q "/submit_order"; do
  if (( SECONDS > 120 )); then
    echo "[smoke] FAIL: 等待 /submit_order 超时 (120s)"
    exit 1
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] /submit_order 可用 (${SECONDS}s)"

# 额外等待 /delivery_status topic 有 publisher，确认 executor 已激活
echo "[smoke] 等待 delivery_executor 激活..."
SECONDS=0
while ! ros2 topic info /delivery_status 2>/dev/null | grep -q "Publisher count: [1-9]"; do
  if (( SECONDS > 60 )); then
    echo "[smoke] WARN: 等待 /delivery_status publisher 超时，继续执行"
    break
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] executor 就绪 (${SECONDS}s)"

# 提交订单
echo "[smoke] 提交订单 station_A -> station_C..."
ros2 service call /submit_order delivery_interfaces/srv/SubmitOrder \
  "{order: {order_id: 'smoke_001', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}" \
  2>&1 | head -5

# 轮询 /delivery_status，在关键阶段自动确认
echo "[smoke] 轮询配送状态..."
LOAD_CONFIRMED=false
UNLOAD_CONFIRMED=false
SECONDS=0

while (( SECONDS < TIMEOUT )); do
  # 获取最新状态
  STATUS_LINE=$(timeout 5 ros2 topic echo /delivery_status --once --no-arr 2>/dev/null | grep -E "^state:" | head -1 || true)
  STATE=$(echo "$STATUS_LINE" | grep -oP '^\d+$' || echo "-1")

  case "$STATE" in
    2) # STATE_WAITING_LOAD
      if ! $LOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_LOAD, 发送 confirm_load..."
        ros2 service call /confirm_load std_srvs/srv/Trigger 2>&1 | head -3
        LOAD_CONFIRMED=true
      fi
      ;;
    4) # STATE_WAITING_UNLOAD
      if ! $UNLOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_UNLOAD, 发送 confirm_unload..."
        ros2 service call /confirm_unload std_srvs/srv/Trigger 2>&1 | head -3
        UNLOAD_CONFIRMED=true
      fi
      ;;
    5) # STATE_COMPLETE
      echo "[smoke] PASS: 配送完成 (state=5), 耗时 ${SECONDS}s"
      exit 0
      ;;
    6) # STATE_FAILED
      echo "[smoke] FAIL: 配送失败 (state=6)"
      exit 1
      ;;
  esac

  sleep "$POLL_INTERVAL"
done

echo "[smoke] FAIL: 超时 (${TIMEOUT}s)"
exit 1
