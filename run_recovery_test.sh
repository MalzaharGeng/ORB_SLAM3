#!/usr/bin/env bash
# Launch the existing D435i recovery workflow from any working directory.
set -Eeuo pipefail

repo_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
build_dir=$(realpath -m -- "${BUILD_DIR:-$repo_dir/build}")
mode=live
debug=0
check_only=0
build_first=0
px4_config=""
display_mode=local
terminal_control=on
software_rendering=1
config_file="$repo_dir/config/recovery_local.conf"

usage() {
    cat <<'EOF'
用法：./run_recovery_test.sh [选项]

默认：连接 D435i，打开 GUI，手动测试存图 → 重置 → 再初始化 → 读档。
  --replay     使用已有录制离线回放，自动执行各阶段，无需相机
  --debug      使用 GDB 启动，异常退出时记录所有线程回溯
  --build      启动前编译对应程序（BUILD_JOBS 默认 2）
  --headless   关闭 GUI；实机通过终端指令推进各阶段
  --gui        在当前桌面显示 GUI（默认）
  --ssh-x11    使用 SSH X11 转发显示到 Windows 等客户端
  --config FILE  读取显示/终端配置，命令行选项优先
  --px4-config FILE  启用 PX4 桥接；回放固定隔离到 domain 42 的 /vio_test
  --console / --no-console  开启/关闭终端控制（实机无 GUI 时必须开启）
  --check      只检查文件、依赖和启动命令，不打开相机或 GUI
  -h, --help   显示帮助

示例：
  ./run_recovery_test.sh
  ./run_recovery_test.sh --debug
  ./run_recovery_test.sh --headless
  ./run_recovery_test.sh --config config/recovery_ssh_windows.conf
  ./run_recovery_test.sh --replay --debug
  ./run_recovery_test.sh --build --check

构建目录：BUILD_DIR（默认仓库内 build/）；可用 RECORDING_DIR 指定回放数据。
日志和存档：仓库内 recordings/test_runs/ 下，每次创建独立目录。
实机测试没有固定结束时长；完成后输入 quit 或点击 GUI 的 Stop 退出。
EOF
}

die() { printf '错误：%s\n' "$*" >&2; exit 1; }
# Config is data, not shell code. Apply it first so explicit CLI flags win.
arguments=("$@")
for ((i=0;i<${#arguments[@]};++i)); do
    case "${arguments[i]}" in
        -h|--help) usage;exit 0 ;;
        --config)
            ((i+1<${#arguments[@]})) || die "--config 后需要文件路径。"
            config_file=${arguments[i+1]};i=$((i+1)) ;;
    esac
done
[[ -r "$config_file" ]] || die "无法读取配置：$config_file"
config_file=$(realpath -- "$config_file")
while IFS= read -r config_line || [[ -n "$config_line" ]]; do
    config_line=${config_line%$'\r'}
    [[ "$config_line" =~ ^[[:space:]]*(#.*)?$ ]] && continue
    if [[ "$config_line" =~ ^[[:space:]]*([A-Z_]+)[[:space:]]*=[[:space:]]*([a-z0-9-]+)[[:space:]]*$ ]]; then
        case "${BASH_REMATCH[1]}" in
            DISPLAY_MODE) display_mode=${BASH_REMATCH[2]} ;;
            TERMINAL_CONTROL) terminal_control=${BASH_REMATCH[2]} ;;
            SOFTWARE_RENDERING) software_rendering=${BASH_REMATCH[2]} ;;
            *) die "未知配置项：${BASH_REMATCH[1]}" ;;
        esac
    else
        die "配置格式错误（仅支持 KEY=value）：$config_line"
    fi
done < "$config_file"
while (($#)); do
    case "$1" in
        --replay) mode=replay ;;
        --debug) debug=1 ;;
        --build) build_first=1 ;;
        --headless|--no-gui) display_mode=headless ;;
        --gui) display_mode=local ;;
        --ssh-x11) display_mode=ssh-x11 ;;
        --console) terminal_control=on ;;
        --no-console) terminal_control=off ;;
        --config) shift ;;
        --px4-config)
            (($#>=2)) || die "--px4-config 后需要文件路径。"
            px4_config=$(realpath -- "$2");shift
            [[ -r "$px4_config" ]] || die "无法读取 PX4 配置：$px4_config" ;;
        --check) check_only=1 ;;
        -h|--help) usage; exit 0 ;;
        *) die "未知参数：$1。使用 --help 查看用法。" ;;
    esac
    shift
done
case "$display_mode" in local|headless|ssh-x11) ;; *) die "DISPLAY_MODE 必须为 local、headless 或 ssh-x11。" ;; esac
case "$terminal_control" in on|off) ;; *) die "TERMINAL_CONTROL 必须为 on 或 off。" ;; esac
case "$software_rendering" in 0|1) ;; *) die "SOFTWARE_RENDERING 必须为 0 或 1。" ;; esac
headless=0
[[ "$display_mode" == headless ]] && headless=1
if ((headless)) && [[ "$mode" == live && "$terminal_control" == off ]]; then
    die "实机无 GUI 模式必须启用终端控制，不能使用 --no-console。"
fi
if [[ "$display_mode" == ssh-x11 ]]; then
    # DISPLAY and XAUTHORITY belong to ssh; never replace them with the PC IP.
    export PANGOLIN_WINDOW_URI=x11://
    export QT_QPA_PLATFORM=xcb
    export QT_X11_NO_MITSHM=1
    export EGL_PLATFORM=x11
    export LIBGL_ALWAYS_SOFTWARE="$software_rendering"
    export LIBGL_DRI3_DISABLE=1
    unset WAYLAND_DISPLAY
fi

# Preserve ROS/RealSense paths but avoid the MVS copy of libusb. This only
# changes this launcher and its children, never the user's shell settings.
runtime_libs="$build_dir/lib"
IFS=: read -r -a inherited_libs <<< "${LD_LIBRARY_PATH-}"
for entry in "${inherited_libs[@]}"; do
    case "$entry" in
        ''|/opt/MVS|/opt/MVS/*) continue ;;
        *) runtime_libs+=":$entry" ;;
    esac
done
export LD_LIBRARY_PATH="$runtime_libs"

vocabulary="$repo_dir/Vocabulary/ORBvoc.txt"
recording=$(realpath -m -- "${RECORDING_DIR:-$repo_dir/recordings/d435i_recovery_verified_20260918_1654}")
if [[ "$mode" == live ]]; then
    target=stereo_inertial_realsense_D435i
    program="$build_dir/Examples/Stereo-Inertial/$target"
    settings="$repo_dir/Examples/Stereo-Inertial/RealSense_D435i.yaml"
else
    target=d435i_recovery_replay
    program="$build_dir/tests/$target"
    settings="$recording/settings.yaml"
    for input in stereo.csv gyro.csv accel.csv reset_at.txt; do
        [[ -s "$recording/combined/$input" ]] || die "缺少回放输入：$recording/combined/$input"
    done
    [[ -s "$recording/export/left/0.png" && -s "$recording/second_capture/export/left/0.png" ]] ||
        die "缺少导出的回放图像，请查看 tests/README.md。"
    export ORB_REPLAY_STRIDE=2
    if ((headless)); then
        unset ORB_REPLAY_VIEWER
    else
        export ORB_REPLAY_VIEWER=1
    fi
    if [[ "$terminal_control" == on ]]; then export ORB_REPLAY_CONSOLE=1; else unset ORB_REPLAY_CONSOLE; fi
fi
if ((!build_first)); then
    [[ -s "$vocabulary" ]] || die "缺少 ORB 词典：$vocabulary"
fi
[[ -s "$settings" ]] || die "缺少配置：$settings"
# Lock before compiling as well: replacing the shared library during another
# test would invalidate that running process. A read-only check needs no lock.
output_root="$repo_dir/recordings/test_runs"
if ((!check_only || build_first)); then
    mkdir -p -- "$output_root"
    exec 9>"$output_root/.launcher.lock"
    flock -n 9 || die "已有测试在运行。请先结束它：输入 quit / 点击 Stop，或等待回放结束。"
fi
if ((build_first)); then
    if [[ ! -s "$vocabulary" ]]; then
        tar -xf "$repo_dir/Vocabulary/ORBvoc.txt.tar.gz" -C "$repo_dir/Vocabulary"
    fi
    cmake_options=(-DORB_SLAM3_BUILD_TESTS=ON)
    [[ -z "$px4_config" ]] || cmake_options+=(-DORB_SLAM3_WITH_PX4=ON)
    cmake -S "$repo_dir" -B "$build_dir" "${cmake_options[@]}"
    cmake --build "$build_dir" --target "$target" --parallel "${BUILD_JOBS:-2}"
fi
[[ -x "$program" ]] || die "程序未编译：$program。请加 --build 后重试。"
if ((debug)); then
    command -v gdb >/dev/null || die "未找到 gdb，无法使用 --debug。"
fi
dependencies=$(ldd "$program") || die "无法检查程序动态库。"
if [[ "$dependencies" == *"not found"* ]]; then
    printf '%s\n' "$dependencies" >&2
    die "存在缺失的动态库。"
fi

command_line=("$program" "$vocabulary" "$settings")
if [[ "$mode" == live ]]; then
    command_line+=(d435i_recovery)
    if ((headless)); then command_line+=(--headless); else command_line+=(--gui); fi
    if [[ "$terminal_control" == on ]]; then command_line+=(--console); else command_line+=(--no-console); fi
else
    # Replaced with an absolute output directory immediately before launching.
    command_line+=("$recording/combined" '<本次测试目录>')
fi
[[ -z "$px4_config" ]] || command_line+=(--px4-config "$px4_config")
if ((check_only)); then
    printf '模式：%s\n文件与动态库检查通过（未检查相机连接或 SLAM 初始化）。\n命令：' "$mode"
    printf '%q ' "${command_line[@]}"
    printf '\nGDB：%s；显示：%s；终端控制：%s\n配置：%s\nDISPLAY：%s\n' "$debug" "$display_mode" "$terminal_control" "$config_file" "${DISPLAY-未设置}"
    if [[ "$display_mode" == ssh-x11 ]]; then
        printf 'X11 后端：Pangolin=x11、OpenCV=xcb；软件渲染=%s；未验证远端窗口。\n' "$software_rendering"
    fi
    printf '%s\n' "$dependencies" | awk '/libusb|librealsense|libORB_SLAM3/ {print}'
    exit 0
fi
if [[ "$display_mode" == ssh-x11 ]]; then
    [[ -n "${SSH_CONNECTION-}" && -n "${DISPLAY-}" ]] ||
        die "请从 Windows 的 MobaXterm 开启 X11 转发后 SSH 登录，再启动此配置。不要手动设置 DISPLAY。"
fi
if ((!headless)) && [[ -z "${DISPLAY-}" ]]; then
    die "DISPLAY 未设置；无窗口运行请加 --headless，远程显示请使用 SSH X11 转发。"
fi
if [[ "$display_mode" == ssh-x11 ]] && command -v xdpyinfo >/dev/null; then
    timeout 8s xdpyinfo >/dev/null 2>&1 || die "SSH DISPLAY 无法连接，请检查 Windows X server 和 SSH 的 X11 转发。"
fi

run_dir=$(mktemp -d "$output_root/$(date +%Y%m%d_%H%M%S)_${mode}_XXXXXX")
cp -- "$settings" "$run_dir/settings.yaml"
cp -- "$config_file" "$run_dir/launcher.conf"
if [[ -n "$px4_config" ]]; then
    cp -- "$px4_config" "$run_dir/px4.yaml"
    command_line[${#command_line[@]}-1]="$run_dir/px4.yaml"
fi
printf 'DISPLAY_MODE=%s\nTERMINAL_CONTROL=%s\nSOFTWARE_RENDERING=%s\n' "$display_mode" "$terminal_control" "$software_rendering" > "$run_dir/effective_launcher.conf"
command_line[2]="$run_dir/settings.yaml"
if [[ "$mode" == replay ]]; then
    command_line[4]="$run_dir"
fi
if ((debug)); then
    command_line=(gdb -q -nx -batch --return-child-result
        -ex 'set pagination off' -ex 'set print thread-events off'
        -ex 'handle SIGINT nostop print pass' -ex 'handle SIGTERM nostop print pass'
        -ex run -ex 'thread apply all bt 15' --args "${command_line[@]}")
fi
{
    printf '模式：%s\n显示：%s；终端控制：%s\n配置：%s\nDISPLAY：%s\n测试目录：%s\n命令：' "$mode" "$display_mode" "$terminal_control" "$config_file" "${DISPLAY-未设置}" "$run_dir"
    printf '%q ' "${command_line[@]}"
    printf '\n'
    if [[ "$mode" == live ]]; then
        cat <<'EOF'
操作顺序：
1. 提供运动激励，看到 end VIBA 2 且正常跟踪后，输入 first（或点击 First Init Done）。
2. 等待 Atlas save succeeded，再输入 reset（或点击 Start Second Map）。
3. 再次看到 end VIBA 2 后，输入 second（或点击 Second Init Done）。
4. 等待 SECOND_READY，再输入 load，分别检查 FILE LOAD SUCCESS 和 ALIGNMENT SUCCESS。
5. 输入 status / pose 查看状态和位置；完成后输入 quit，或点击 Stop。
EOF
    else
        printf '离线回放自动执行各阶段；图像步长为 2，保留期间全部 IMU。可输入 status / pose / quit。\n'
    fi
} | tee "$run_dir/launch.txt"

# A separate working directory keeps the GUI's fixed archive name from
# overwriting the previous test's orbslam3_first_map.osa.
cd -- "$run_dir"
set +e
"${command_line[@]}" 2>&1 | tee "$run_dir/run.log"
pipeline_status=("${PIPESTATUS[@]}")
set -e
result=${pipeline_status[0]}
if ((result == 0 && pipeline_status[1] != 0)); then result=${pipeline_status[1]}; fi
# The upstream live example returns 0 when no camera is connected.
if grep -Fq 'No device connected' "$run_dir/run.log"; then result=1; fi
if [[ "$mode" == replay ]] && ! grep -Eq '^RECOVERY_REPLAY_PASS|^REPLAY_STOPPED_BY_USER$' "$run_dir/run.log"; then result=1; fi
printf '\n程序退出码：%s\n日志：%s/run.log\n存档及结果目录：%s\n' "$result" "$run_dir" "$run_dir"
exit "$result"
