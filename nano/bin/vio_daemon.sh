#!/bin/bash
# 无人值守 VIO 守护进程：MicroXRCE-DDS Agent + headless ORB-SLAM3
#   - 不依赖图形桌面，可在开机后直接运行
#   - SLAM 退出或卡死自动重启
#   - 相机/USB 抖动时自愈后重试
# 日志：~/vio_daemon.log   单次会话日志：~/ORB_SLAM3/recordings/test_runs/<时间>/
set -u

H="$HOME"
LOG="$H/vio_daemon.log"
AGENT_BIN="$H/.cache/orbslam3-px4/agent_install/bin/MicroXRCEAgent"
AGENT_LIB="$H/.cache/orbslam3-px4/agent_install/lib"
AGENT_LOG="$H/vio_agent.log"
ROS_SETUP="/opt/ros/humble/setup.bash"
MSGS_SETUP="$H/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash"
REPO="$H/ORB_SLAM3"
PIO_CFG="${VIO_PX4_CONFIG:-config/decxin_px4.yaml}"
# 采集入口：stereo_uvc（UVC 双目，纯 Stereo）或 d435i（librealsense）
VIO_ENTRY="${VIO_ENTRY:-stereo_uvc}"
# Linux comm is capped at 15 bytes. Match the selected capture entry.
SLAM_COMM=stereo_inertial
[ "$VIO_ENTRY" = stereo_uvc ] && SLAM_COMM=stereo_uvc
INREC_DIR="$H/inrec"            # 内录 CSV 落地目录
INREC_PY="$H/vio_inrec.py"      # 同步内录脚本（位姿 vs 飞控融合）

RESTART_DELAY="${RESTART_DELAY:-10}"    # SLAM 退出后等待秒数
STALL_LIMIT="${STALL_LIMIT:-90}"        # 桥接日志连续多少秒不增长判定为卡死
BAUD="${BAUD:-921600}"

say() { echo "[$(date '+%F %T')] $*" | tee -a "$LOG"; }

# 单实例：pidfile 比 flock 稳妥，子进程不会继承锁导致重启后自锁。
# 但 pidfile 有盲区：被 kill -9 杀掉时不执行 trap，文件里会留下死 pid，
# 新实例检查到"旧进程已死"就放行 —— systemd restart 若没杀干净子进程，
# 就会同时跑两个 daemon，两者各自的相机自愈逻辑会互相打断（实测踩过）。
# 因此再按进程名兜底查一次。
PIDFILE="$H/.vio_daemon.pid"
running=""
if [ -f "$PIDFILE" ]; then
    oldpid=$(cat "$PIDFILE" 2>/dev/null || true)
    if [ -n "${oldpid:-}" ] && [ "$oldpid" != "$$" ] && kill -0 "$oldpid" 2>/dev/null; then
        running="$oldpid"
    fi
fi
if [ -z "$running" ]; then
    # 兜底：pidfile 可能被 kill -9 杀掉后残留失效，按进程名再查一次。
    # 注意不能用 pgrep -f "vio_daemon.sh" —— 那条命令自身的 cmdline 也含这个字符串，
    # 会匹配到 pgrep 自己，导致永远"已有人在跑"。改为只挑名为 bash 的进程再核对 cmdline。
    for p in $(pgrep -x bash 2>/dev/null); do
        [ "$p" = "$$" ] && continue
        [ "$p" = "${PPID:-0}" ] && continue
        case "$(tr '\0' ' ' <"/proc/$p/cmdline" 2>/dev/null)" in
            *vio_daemon.sh*) running="$p"; break ;;
        esac
    done
fi
if [ -n "$running" ]; then
    # 不能直接 exit：unit 是 Type=simple + Restart=always，退出会被 systemd 20 秒后
    # 再次拉起，形成重启风暴（实测每 20 秒刷一条"已有守护进程在运行"）。
    # 这里挂住等对方结束再接管，服务始终保持 active。
    say "已有守护进程 $running 在运行，本实例待命接管"
    while kill -0 "$running" 2>/dev/null; do sleep 10; done
    say "原守护进程已退出，本实例接管"
    pgrep -x "$SLAM_COMM" >/dev/null 2>&1 && { pkill -9 -x "$SLAM_COMM" 2>/dev/null; sleep 2; }
fi
echo $$ >"$PIDFILE"
trap 'rm -f "$PIDFILE"' EXIT
stop_runtime() {
    trap '' INT TERM
    say "收到停止信号，结束采集与内录，不进入重启循环"
    [ -n "${watcher:-}" ] && kill "$watcher" 2>/dev/null
    pkill -INT -x "$SLAM_COMM" 2>/dev/null
    local n=0
    while pgrep -x "$SLAM_COMM" >/dev/null 2>&1 && [ "$n" -lt 12 ]; do
        sleep 1; n=$((n + 1))
    done
    pkill -KILL -x "$SLAM_COMM" 2>/dev/null
    [ -n "${runner:-}" ] && kill -TERM "$runner" 2>/dev/null
    if [ -n "${inrec_pid:-}" ]; then
        kill -TERM "$inrec_pid" 2>/dev/null
        local record_wait=0
        while kill -0 "$inrec_pid" 2>/dev/null && [ "$record_wait" -lt 12 ]; do
            sleep 1;record_wait=$((record_wait+1))
        done
    fi
    pkill -TERM -x MicroXRCEAgent 2>/dev/null
    exit 0
}
trap stop_runtime INT TERM

# headless 实机模式强制要求 --console，而 console 读到 stdin EOF 会主动退出。
# 用一个 daemon 自己持有写端的 FIFO 顶住 stdin，保证永不 EOF 且不产生多余进程。
VIO_FIFO="$H/.vio_stdin.fifo"
rm -f "$VIO_FIFO"
mkfifo "$VIO_FIFO" || { say "建不了 FIFO $VIO_FIFO"; exit 1; }

find_serial() {
    for c in /dev/serial/by-id/*CP2102* /dev/serial/by-id/*Silicon* /dev/ttyUSB0 /dev/ttyACM0; do
        [ -e "$c" ] && { echo "$c"; return 0; }
    done
    return 1
}

wait_camera() {
    local n=0 v h tries=0 res=""
    # 上一轮 SLAM 若还在退出，usbfs 接口仍被它占着，此时 video 必然是 2、hidraw 消失。
    # 这是"占用态"不是"故障态"，必须先把它彻底终止再判断，否则会误判故障去做 USB 复位，
    # 而复位会把正在工作的相机打断，形成"复位→SLAM 崩→再复位"的死循环（实测踩过两次）。
    if pgrep -x "$SLAM_COMM" >/dev/null 2>&1; then
        say "上一轮 SLAM 仍在运行，先终止并等接口释放"
        pkill -9 -x "$SLAM_COMM" 2>/dev/null
        sleep 3
    fi
    if [ -f "$H/.vio_camera_recover" ]; then
        say "上次图像采集无进展，执行定向 D435i 恢复"
        sudo -n /usr/local/sbin/vio-camera-prepare >>"$LOG" 2>&1 || true
    fi
    while [ "$n" -lt 12 ]; do
        v=$(ls /dev/video* 2>/dev/null | wc -l)
        h=$(ls /dev/hidraw* 2>/dev/null | wc -l)
        # D435i 枚举 6 个 video 节点 + 1 个 hidraw；德创信 UVC 双目只有 2 个 video
        # 节点、没有 hidraw，所以就绪判据要跟着采集入口走。
        if [ "$VIO_ENTRY" = "stereo_uvc" ]; then
            ready_ok=0; [ "$v" -ge 2 ] && ready_ok=1
        else
            ready_ok=0; [ "$v" -ge 6 ] && [ "$h" -ge 1 ] && ready_ok=1
        fi
        if [ "$ready_ok" -eq 1 ]; then
            say "相机就绪 video=$v hidraw=$h"
            # 节点出现 ≠ UVC 控制通道就绪。刚枚举完立刻开流会 get_xu(EAGAIN) 直接退出，
            # 所以留一个稳定窗口；CAM_SETTLE 可覆盖。
            sleep "${CAM_SETTLE:-5}"
            return 0
        fi
        n=$((n + 1)); sleep 2

        # librealsense 退出后若把 usbfs 接口据着不放，video 会掉到 2、hidraw 消失，
        # 此时干等永远不会齐，只能靠一次重新枚举解开。最多试 2 次。
        if [ "$n" -eq 12 ] && [ "$tries" -lt 2 ]; then
            # 复位前再确认一次没有 SLAM 占用，避免把正常占用当故障复位
            if pgrep -x "$SLAM_COMM" >/dev/null 2>&1; then
                say "SLAM 仍占用相机（video=$v hidraw=$h），跳过复位继续等待"
                n=0; continue
            fi
            tries=$((tries + 1))
            say "相机未齐且无 SLAM 占用（video=$v hidraw=$h），第 ${tries} 次 USB 复位"
            res=$(sudo -n /usr/local/sbin/vio-camera-prepare 2>&1 | tail -1)
            say "USB 复位结果：${res:-无输出}"
            sleep 6
            n=0
        fi
    done
    say "相机始终没齐（video=$v hidraw=$h），照常启动让上层重试"
    return 1
}

start_agent() {
    local dev="$1" i try
    pkill -x MicroXRCEAgent 2>/dev/null
    sleep 1
    # 上电初期飞控（PX4）自身还没启动完，串口没有数据可读，握手会静默失败。
    # Agent 自己不会重试，只有桥接会重试订阅，所以必须在这里整只重启。
    for try in 1 2 3; do
        : >"$AGENT_LOG"
        setsid nohup env LD_LIBRARY_PATH="$AGENT_LIB" \
            "$AGENT_BIN" serial --dev "$dev" -b "$BAUD" \
            >"$AGENT_LOG" 2>&1 </dev/null &
        for i in $(seq 1 10); do
            grep -Eq "session (re-)?established" "$AGENT_LOG" 2>/dev/null && { say "DDS Agent 已连上飞控 ($dev)"; return 0; }
            sleep 1
        done
        if [ "$try" -lt 3 ]; then
            say "DDS Agent 第 ${try} 次握手失败，重启 Agent 重试"
            pkill -x MicroXRCEAgent 2>/dev/null
            sleep 3
        fi
    done
    say "DDS Agent 暂未握手，回到启动循环重试（不依赖网络）"
    return 1
}

# 图像处理心跳证明采集和跟踪确实有进展；PX4心跳只证明桥接活着。
watch_stall() {
    local log="$1" last=-1 cur=0 same=0 pcpu="?" cpu perf=0 last_perf=-1 capture_stall=0
    while :; do
        sleep 15
        cur=0
        [ -f "$log" ] && cur=$(wc -l <"$log" 2>/dev/null || echo 0)
        perf=$(grep -acE '\[VIO_PERF\]|\[STAT\]' "$log" 2>/dev/null || true)
        perf="${perf:-0}"
        if [ "$perf" -eq "$last_perf" ]; then
            capture_stall=$((capture_stall + 15))
        else
            capture_stall=0; last_perf=$perf
        fi
        if [ "$capture_stall" -ge 45 ]; then
            say "图像处理心跳45秒无进展，保存异常标记并重启采集"
            printf 'capture_stall\n' > "$(dirname "$log")/freeze_images"
            touch "$H/.vio_camera_recover"
            sleep 1
            pkill -INT -x "$SLAM_COMM" 2>/dev/null
            sleep 5
            pkill -9 -x "$SLAM_COMM" 2>/dev/null
            return 0
        fi
        # 判活不能只看日志行数：等待 IMU 初始化时日志是节流打印的，
        # 几十秒不增长很常见，只看行数会把"正在正常等待初始化"误判为卡死。
        # 再把进程累计 CPU 时间作为第二路活动证据，两者都不变才认为真卡死。
        cpu=$(ps -o cputime= -C "$SLAM_COMM" 2>/dev/null | head -1 | tr -d ' ')
        cpu="${cpu:-gone}"
        if [ "$cur" -eq "$last" ] && [ "$cpu" = "$pcpu" ]; then
            same=$((same + 15))
            if [ "$same" -ge "$STALL_LIMIT" ]; then
                say "检测到卡死（${same}s 日志与CPU均无变化），结束本次会话准备重启"
                pkill -x "$SLAM_COMM" 2>/dev/null
                sleep 3
                pkill -9 -x "$SLAM_COMM" 2>/dev/null
                return 0
            fi
        else
            same=0; last=$cur; pcpu=$cpu
        fi
    done
}


# Telemetry runs across SLAM restarts; per-SLAM exact state is recorded separately.
inrec_pid=""
inrec_part=0
inrec_session="$(date +%Y%m%d_%H%M%S)_$$"
inrec_bridge="$INREC_DIR/bridge_${inrec_session}.log"
start_inrec() {
    [ -f "$INREC_PY" ] || { say "完整内录脚本缺失，拒绝无内录启动"; return 1; }
    mkdir -p "$INREC_DIR"
    inrec_part=$((inrec_part+1))
    inrec_csv="$INREC_DIR/inrec_${inrec_session}_part${inrec_part}.csv"
    inrec_log="$INREC_DIR/inrec_${inrec_session}_part${inrec_part}.log"
    python3 "$INREC_PY" --bridge-log "$inrec_bridge" --out "$inrec_csv" >"$inrec_log" 2>&1 &
    inrec_pid=$!
    local attempt
    for attempt in 1 2 3 4 5 6 7 8 9 10; do
        kill -0 "$inrec_pid" 2>/dev/null || { say "内录启动失败，见 $inrec_log"; return 1; }
        if grep -q '^INREC_READY' "$inrec_log"; then
            say "完整内录已就绪 -> $inrec_csv"
            return 0
        fi
        sleep 1
    done
    kill -TERM "$inrec_pid" 2>/dev/null
    say "内录启动超时，拒绝无内录运行"
    return 1
}

say "=== 守护进程启动 ==="

while :; do
    dev=$(find_serial) || { say "找不到飞控串口，20 秒后重试"; sleep 20; continue; }

    wait_camera
    if ! start_agent "$dev"; then
        sleep 10
        continue
    fi

    # ROS 的 setup.bash 会引用未定义变量，在 set -u 下会直接终止脚本，必须临时放宽
    set +u
    # shellcheck disable=SC1090
    source "$ROS_SETUP" >/dev/null 2>&1
    # shellcheck disable=SC1090
    source "$MSGS_SETUP" >/dev/null 2>&1
    set -u

    if [ -z "${inrec_pid:-}" ] || ! kill -0 "$inrec_pid" 2>/dev/null; then
        start_inrec || { sleep "$RESTART_DELAY"; continue; }
    fi
    # New VIO processes require fresh disarmed/landed telemetry. Initialization
    # may continue after manual takeoff, but process failures cannot re-anchor
    # external vision while the aircraft remains armed.
    if [ -n "${VIO_IMU_CONFIG:-}" ]; then
        gate="$H/vio_restart_gate.py"
        [ -f "$gate" ] || { say "VIO restart gate missing";sleep 5;continue; }
        gate_ready=0
        for gate_try in 1 2 3 4 5 6 7 8 9 10; do
            if ! kill -0 "$inrec_pid" 2>/dev/null; then start_inrec || break;fi
            if python3 "$gate" "$inrec_csv.summary.json" >>"$LOG" 2>&1; then
                gate_ready=1;break
            fi
            sleep 2
        done
        if [ "$gate_ready" -ne 1 ]; then
            say "Waiting for fresh disarmed/landed status; no airborne VIO restart"
            sleep 3
            continue
        fi
    fi
    cd "$REPO" || { say "进不去 $REPO"; sleep 30; continue; }

    # Bound image storage across sessions and recover a crashed/unclean ring.
    # This touches only marked image rings/events; telemetry CSVs are untouched.
    if [ -f "$H/vio_image_retention.py" ]; then
        python3 "$H/vio_image_retention.py" >>"$LOG" 2>&1 || say "图像保留策略告警，继续 VIO"
    fi
    export VIO_EVENT_DIR="$H/vio_image_events"
    say "启动 SLAM（headless，完整状态内录已就绪；图像按显式配置录制）"
    # 第 8 号 fd 以读写方式打开 FIFO，daemon 自己持着写端，读端永不 EOF
    exec 8<>"$VIO_FIFO"
    VIO_ENTRY="$VIO_ENTRY" ./run_recovery_test.sh --headless --px4-config "$PIO_CFG" <"$VIO_FIFO" 8>&- &
    runner=$!

    # 取本次会话目录，交给卡死监控。
    # 不能用 ls -dt 猜：nano 断电后时钟会倒退或跳到"未来"，会选中一个早已停止写入的
    # 旧目录，看门狗于是盯着死文件、每 90 秒误杀一次 SLAM（正好打断初始化动作）。
    # 改为从 SLAM 进程的命令行参数里读它真正在用的会话目录。
    run_dir=""
    sleep 8
    for _try in 1 2 3 4 5 6 7 8; do
        _p=$(pgrep -x "$SLAM_COMM" 2>/dev/null | head -1)
        if [ -n "${_p:-}" ]; then
            _d=$(tr '\0' '\n' <"/proc/$_p/cmdline" 2>/dev/null | grep -o "$REPO/recordings/test_runs/[^/]*" | head -1)
            if [ -n "${_d:-}" ]; then run_dir="$_d/"; break; fi
        fi
        sleep 3
    done
    if [ -z "${run_dir:-}" ]; then
        say "取不到本会话目录，退化为按修改时间挑选"
        run_dir=$(ls -dt "$REPO"/recordings/test_runs/*/ 2>/dev/null | head -1)
    fi
    watcher=""
    if [ -n "${run_dir:-}" ]; then
        watch_stall "$run_dir/run.log" &
        watcher=$!

        ln -sfn -- "$run_dir/run.log" "$inrec_bridge"

    fi

    while kill -0 "$runner" 2>/dev/null; do
        if ! kill -0 "$inrec_pid" 2>/dev/null; then
            say "内录进程退出，立即启动新分段并保留原分段状态"
            if ! start_inrec; then
                say "完整内录恢复失败，结束本次 SLAM"
                pkill -INT -x "$SLAM_COMM" 2>/dev/null
                break
            fi
        fi
        sleep 2
    done
    wait "$runner"
    rc=$?
    [ -n "${watcher:-}" ] && kill "$watcher" 2>/dev/null
    # Keep telemetry alive while SLAM/Agent recover; gaps remain visible.
    exec 8>&- 2>/dev/null

    say "SLAM 退出（rc=$rc），${RESTART_DELAY} 秒后重启"
    pkill -x "$SLAM_COMM" 2>/dev/null
    sleep 2
    pkill -9 -x "$SLAM_COMM" 2>/dev/null
    sleep "$RESTART_DELAY"
done
