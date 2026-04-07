#!/usr/bin/env bash
# smoke_test.sh — 端到端冒烟测试：提交订单 → 自动确认 → 验证完成
# 用法: bash scripts/smoke_test.sh
# 超时: 420 秒 (wall-clock)
#
# Bash 约定说明：
# - 本脚本要求用 bash 执行，不要用 sh。很多语法（如 [[ ... ]] / (( ... ))）是 bash 扩展。
# - 函数写法 foo() { ... } 中的花括号不是“代码块对象”，只是 shell 的命令分组语法。
# - 注释里会顺手解释常见 shell 写法，方便后续自己读。
#
# set -euo pipefail 是“严格模式”：
# -e: 某条命令返回非 0 时，默认立刻退出脚本
# -u: 使用未定义变量时报错
# -o pipefail: 管道中任意一段失败，整个管道都算失败
# 这能减少“脚本默默带错往下跑”的情况。
set -euo pipefail

# 普通变量。这里没用 readonly，是因为它们不是常量中的“协议字段”，只是脚本配置。
TIMEOUT=420
POLL_INTERVAL=2
DEMO_PID=""
CLEANUP_DONE=false
ORDER_ID="smoke_001"

# readonly 表示“只读常量”，后面再赋值会报错。
# DeliveryStatus.msg 状态枚举
readonly STATE_WAITING_LOAD=2
readonly STATE_GOING_TO_DROPOFF=3
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
# 函数定义语法：函数名() { 命令... }
kill_stale_processes() {
  echo "[smoke] 清理残留进程..."
  # 先杀 launch 父进程（会级联终止其子进程组）
  # `|| true` 的意思是：就算 pkill 没找到进程返回非 0，也不要让 set -e 终止脚本。
  pkill -f "ros2 launch delivery_bringup" 2>/dev/null || true
  # 杀本项目特有的节点（追加 --ros-args 限定，避免误杀测试二进制或其他工作区实例）
  pkill -f "delivery_manager.*--ros-args" 2>/dev/null || true
  pkill -f "delivery_executor.*--ros-args" 2>/dev/null || true
  pkill -f "delivery_lifecycle_manager.*--ros-args" 2>/dev/null || true
  # Gazebo 仿真进程（用 warehouse 限定，只杀本项目的仿真实例）
  pkill -f "ruby.*gz.*warehouse" 2>/dev/null || true
  pkill -f "gz sim.*warehouse" 2>/dev/null || true
}

cleanup() {
  # 这里的 if $CLEANUP_DONE 不是字符串判断，而是把变量内容当命令执行。
  # 因为变量只会是 true/false，而它们正好是 shell 内置命令：
  # - true  返回 0
  # - false 返回非 0
  if $CLEANUP_DONE; then return; fi
  CLEANUP_DONE=true
  echo "[smoke] 清理中..."
  # 终止后台状态订阅（整个进程组，包括 ros2 topic echo 和 grep）
  # [[ ... ]] 是 bash 的条件判断，比 [ ... ] 更安全，支持更自然的字符串判断。
  # ${STATUS_SUB_PID:-} 表示：如果变量未定义，则展开为空字符串，避免 set -u 报错。
  # kill -0 PID 不会真的杀进程，只是用来探测“这个 PID 是否存在且可访问”。
  if [[ -n "${STATUS_SUB_PID:-}" ]] && kill -0 "$STATUS_SUB_PID" 2>/dev/null; then
    # kill -- -PID 的负号表示“给整个进程组发信号”，不是只杀单个进程。
    # 这里之所以能这样做，是因为前面用 setsid 创建了独立会话/进程组。
    # -- 用来告诉 kill：“后面不是选项了，哪怕它以 - 开头也当参数解析”。
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

# trap 的意思是“在收到某些事件/信号时自动执行指定命令”。
# 这里不管脚本正常结束、Ctrl+C 中断，还是收到 TERM，都会执行 cleanup。
trap cleanup EXIT INT TERM

# 这两行是 shell 里很常见的“脚本所在目录/工作区目录”求法：
# - $0 是当前脚本路径
# - dirname "$0" 取脚本所在目录
# - $(...) 是命令替换：把命令输出塞进变量
SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../ros2_ws" && pwd)"

# 环境准备
# source 会在“当前 shell”里执行脚本，常用于加载环境变量。
# ROS setup 脚本内部会读取某些未定义变量；这里临时关闭 nounset，
# 避免 set -u 让冒烟脚本在环境初始化前就退出。
set +u
source /opt/ros/jazzy/setup.bash
source "$WS_DIR/install/setup.bash"
set -u
export TURTLEBOT3_MODEL=waffle_pi
# 给冒烟测试固定独立的 DDS 域，避免其他 ROS 会话或残留发现信息污染 /clock、service graph。
# 这里不读取用户当前 shell 的 ROS_DOMAIN_ID，目的是让脚本自带隔离性和可重复性。
export ROS_DOMAIN_ID=88

# 启动前先清理旧进程，避免 /clock 多 publisher 污染
kill_stale_processes
sleep 2

echo "[smoke] 启动 demo (headless, gui:=false, rviz:=false)..."
# setsid 会让命令成为新的会话/进程组 leader，后面就能用 kill -- -PID 整组清理。
# 行尾的 & 表示放到后台执行，不阻塞当前脚本。
setsid ros2 launch delivery_bringup demo.launch.py \
  gui:=false \
  rviz:=false &
# $! 是“最近一个后台任务的 PID”。
DEMO_PID=$!

# 等待 /submit_order 服务可用
echo "[smoke] 等待 /submit_order 服务..."
# SECONDS 是 bash 内置变量，赋值为 0 后会开始自动累计脚本运行秒数。
SECONDS=0
# while ! cmd; do ... done: 只要 cmd 失败，就循环。
# grep -q 只关心“有没有匹配”，不输出内容。
while ! ros2 service list 2>/dev/null | grep -q "$SRV_SUBMIT_ORDER"; do
  # (( ... )) 是 bash 算术判断，里面直接写数字表达式，不需要 $。
  if (( SECONDS > 120 )); then
    echo "[smoke] FAIL: 等待 $SRV_SUBMIT_ORDER 超时 (120s)"
    exit 1
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] $SRV_SUBMIT_ORDER 可用 (${SECONDS}s)"

# 等待 /delivery_status topic 有至少 1 个 publisher。
# 2026-04-05 起状态统一由 delivery_manager 对外发布，不再依赖双发布源。
echo "[smoke] 等待配送节点就绪..."
SECONDS=0
while ! ros2 topic list 2>/dev/null | grep -q "^${TOPIC_DELIVERY_STATUS}$"; do
  if (( SECONDS > 60 )); then
    echo "[smoke] WARN: 等待 $TOPIC_DELIVERY_STATUS 话题出现超时，继续执行"
    break
  fi
  sleep "$POLL_INTERVAL"
done
echo "[smoke] 配送节点就绪 (${SECONDS}s)"

# 提交订单（带重试：system_ready_ 可能在服务注册后才置 true）
echo "[smoke] 提交订单 station_A -> station_C..."
SECONDS=0
while (( SECONDS < 60 )); do
  # $(...) 捕获命令输出；2>&1 把 stderr 合并到 stdout，这样错误信息也能一起拿到。
  # 后面的 || true 是为了让“服务调用失败”变成可重试，而不是触发 set -e 直接退出。
  SUBMIT_OUTPUT=$(timeout 10s ros2 service call "$SRV_SUBMIT_ORDER" delivery_interfaces/srv/SubmitOrder \
    "{order: {order_id: '$ORDER_ID', pickup_station: 'station_A', dropoff_station: 'station_C', priority: 0}}" \
    2>&1 || true)
  # 这里的 grep 正则同时兼容不同 CLI 输出格式：accepted=True 或 accepted: True
  if echo "$SUBMIT_OUTPUT" | grep -q "accepted=True\|accepted: True"; then
    echo "$SUBMIT_OUTPUT" | head -5
    echo "[smoke] 订单已接受"
    break
  fi
  # 下面这一段是嵌套命令替换：先从输出里抽取 reason，再塞回 echo。
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
# mktemp 会创建一个唯一的临时文件，并把路径返回给变量。
STATUS_FILE=$(mktemp /tmp/smoke_status.XXXXXX)
# 这里用 bash -c '...' 是因为要把整条管道作为一个整体交给 setsid。
# 单引号包住“大框架”，再通过 '"$VAR"' 的方式把外层变量拼进去，是 shell 里常见写法。
setsid bash -c 'PYTHONUNBUFFERED=1 ros2 topic echo "'"$TOPIC_DELIVERY_STATUS"'" --no-arr 2>/dev/null \
  | stdbuf -oL awk '"'"'
      /^order_id:/ { order_id=$2 }
      /^state:/ { state=$2 }
      /^---$/ {
        if (order_id != "" && state != "") {
          print order_id " " state
          fflush()
        }
        order_id=""
        state=""
      }
    '"'"' \
  > "'"$STATUS_FILE"'"' &
STATUS_SUB_PID=$!

# 轮询 /delivery_status，在关键阶段自动确认
echo "[smoke] 轮询配送状态..."
# 这两个变量依然用 true/false，当成“布尔命令”来判断。
LOAD_CONFIRMED=false
UNLOAD_CONFIRMED=false
SECONDS=0

while (( SECONDS < TIMEOUT )); do
  # 持久订阅文件中每行格式为：<order_id> <state>
  STATUS_LINE=$(tail -1 "$STATUS_FILE" 2>/dev/null || true)
  STATUS_ORDER_ID=$(echo "$STATUS_LINE" | awk '{print $1}')
  STATE=$(echo "$STATUS_LINE" | awk '{print $2}')

  if [[ "$STATUS_ORDER_ID" != "$ORDER_ID" ]]; then
    sleep "$POLL_INTERVAL"
    continue
  fi

  # case ... esac 是 shell 里的多分支匹配，适合根据字符串/常量分情况处理。
  case "$STATE" in
    "$STATE_WAITING_LOAD")
      # if ! $LOAD_CONFIRMED：同样是把变量内容当 true/false 命令执行。
      if ! $LOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_LOAD, 发送 confirm_load..."
        CONFIRM_OUTPUT=$(timeout 10s ros2 service call "$SRV_CONFIRM_LOAD" std_srvs/srv/Trigger 2>&1 || true)
        echo "$CONFIRM_OUTPUT" | sed -n '1,3p'
        LOAD_CONFIRMED=true
      fi
      ;;
    "$STATE_WAITING_UNLOAD")
      if ! $UNLOAD_CONFIRMED; then
        echo "[smoke] 状态=WAITING_UNLOAD, 发送 confirm_unload..."
        CONFIRM_OUTPUT=$(timeout 10s ros2 service call "$SRV_CONFIRM_UNLOAD" std_srvs/srv/Trigger 2>&1 || true)
        echo "$CONFIRM_OUTPUT" | sed -n '1,3p'
        UNLOAD_CONFIRMED=true
      fi
      ;;
    1|"$STATE_GOING_TO_DROPOFF")
      :
      ;;
    "$STATE_COMPLETE")
      echo "[smoke] PASS: 订单 $ORDER_ID 配送完成 (state=$STATE_COMPLETE), 耗时 ${SECONDS}s"
      exit 0
      ;;
    "$STATE_FAILED")
      echo "[smoke] FAIL: 订单 $ORDER_ID 配送失败 (state=$STATE_FAILED)"
      exit 1
      ;;
    *)
      if [[ -n "$STATE" ]]; then
        echo "[smoke] 未识别状态: order_id=$STATUS_ORDER_ID state=$STATE"
      fi
      ;;
  esac

  sleep "$POLL_INTERVAL"
done

# exit 0 表示成功，非 0 表示失败。
# 无论走到哪个 exit，前面的 trap 都会先调用 cleanup。
echo "[smoke] FAIL: 超时 (${TIMEOUT}s)"
exit 1
