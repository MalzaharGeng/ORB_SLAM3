# 测试命令与参数总表

> 来源：Avi 提交 `b9ba5ecf0a490e906ad8cd968d9dd42b39aa8d10`（增加指令集）。本页命令已按 Combat 分支的构建树输出和可配置路径适配；Avi 的真实录制、DDS/SIH 和实机结果仍属于来源记录，不代表本机或 Nano 已验收。Nano 为 ARM64，必须单独构建，不能复制 WSL x86_64 二进制。

先指定克隆位置及构建目录（新终端重新设置）；默认不要求 PX4：

```bash
export ORB_ROOT="${ORB_ROOT:-$HOME/ORB_SLAM3}"
export BUILD_DIR="${BUILD_DIR:-$ORB_ROOT/build}"
cd "$ORB_ROOT"
mkdir -p "$ORB_ROOT/recordings/test_runs"
```

本机历史验证缓存为 `build_sync`，若复用它先设置 `BUILD_DIR="$ORB_ROOT/build_sync"`。回放数据、ROS/PX4、RealSense SDK 是否存在须单独确认。第 3/8 节启用 PX4、安装依赖、连接串口的命令是按需步骤，本次同步没有执行。


按 2026-10-30 当前代码核对。覆盖本项目新增的 D435i 初始化、存图/恢复、导航坐标、帧率、录制及 PX4 通信测试；上游通用 KITTI/EuRoC/TUM 示例不在本表内。

以下命令先进入仓库；各项测试顺序运行，尤其不要在运行 SLAM 时重新编译共享库。

```bash
cd "$ORB_ROOT"
```

## 1. 常用入口速查

| 目的 | 命令 | 相机 / 操作方式 |
|---|---|---|
| 实机 GUI | `./run_recovery_test.sh` | D435i；按钮或终端手动推进 |
| 实机无 GUI | `./run_recovery_test.sh --headless` | D435i；终端手动推进 |
| 实机崩溃回溯 | `./run_recovery_test.sh --headless --debug` | D435i；GDB |
| 已录制数据 GUI 回放 | `./run_recovery_test.sh --replay` | 无需相机；自动推进，stride=2 |
| 已录制数据无 GUI 回放 | `./run_recovery_test.sh --replay --headless` | 无需相机；自动推进，stride=2 |
| 回放崩溃回溯 | `./run_recovery_test.sh --replay --headless --debug` | 无需相机 |
| Windows SSH 显示 | `./run_recovery_test.sh --config config/recovery_ssh_windows.conf` | 在启用 X11 转发的 SSH 会话中执行 |
| 只检查启动条件 | `./run_recovery_test.sh --headless --check` | 不打开相机或窗口 |
| 编译后检查 | `./run_recovery_test.sh --replay --headless --build --check` | 会编译，但不运行 SLAM |
| 完整帧率 / VIBA 时间测试 | `python3 tests/benchmark_recovery.py 输出目录 --stride 1` | 无需相机；详见第 5 节 |
| 查看注册的回归测试 | `ctest --test-dir "$BUILD_DIR" -N` | 只列出，不执行 |

一键入口**不录制原始传感器数据**。每次生成的日志、CSV 和地图存档不是新的录制包；实际录制用第 7 节程序。

## 2. 一键启动参数和阶段指令

```text
./run_recovery_test.sh [选项]
```

| 参数 | 默认 / 含义 |
|---|---|
| 无参数 | 实机、GUI 开启、终端控制开启 |
| `--replay` | 使用固定的已有录制，自动执行存图→重置→再初始化→读档 |
| `--headless` / `--no-gui` | 关闭 GUI；实机仍需终端输入 |
| `--gui` | 使用当前桌面显示 |
| `--ssh-x11` | 使用 SSH X11 转发；保留 SSH 提供的 DISPLAY |
| `--config FILE` | 显示/终端配置；默认 `config/recovery_local.conf`；显式命令行覆盖文件 |
| `--console` / `--no-console` | 开关终端输入；实机无 GUI 时不能关闭终端 |
| `--debug` | GDB 启动，异常退出记录所有线程回溯 |
| `--build` | 先配置并以 `BUILD_JOBS`（默认 2）编译当前选中的实机或回放目标，然后启动 |
| `--check` | 只检查文件、动态库和命令；与 `--build` 组合时仍会编译 |
| `--px4-config FILE` | 启用 PX4 桥接，要求对应编译开关 ON；详见第 8 节 |
| `-h` / `--help` | 打印帮助 |

一键入口目前没有 `--settings`、`--dataset`、`--stride`、`--fast-init` 或 `--viba1` 参数。需要自定义 YAML/数据路径时直接运行第 4 节程序；需要 VIBA 参数扫描时用第 5 节脚本。

SSH GUI 需要 Windows 上运行 X server，并使用支持 X11 转发的 SSH 客户端；MobaXterm 是一种选择。连接后由 SSH 设置 DISPLAY，不要改为 Windows IP 或 Linux 的 `:0`。实际连接步骤见 [REMOTE_AND_CONSOLE.md](REMOTE_AND_CONSOLE.md)。

每条阶段指令输入后回车；等前一项成功再推进，不要一次粘贴全部阶段命令。

| 终端指令 | GUI 对应项 | 触发条件 / 用途 |
|---|---|---|
| `first` / `1` | First Init Done | 第一次 `end VIBA 2`、跟踪正常后保存首次地图 |
| `reset` / `2` | Start Second Map | 等 `Atlas save succeeded` 后全量重置 |
| `second` / `3` | Second Init Done | 第二次 `end VIBA 2`、跟踪正常后确认 |
| `load` / `4` | Read First Map | 等 `SECOND_READY` 后读取首次地图并尝试对齐 |
| `status` | — | 查看阶段、存图状态、导航及读档结果 |
| `pose` | — | 当前 IMU 在初始 IMU 导航系中的 XYZ、距离、四元数 |
| `help` | — | 查看指令 |
| `quit` / `exit` / `q` | Stop | 正常停止 |

`queued` 表示请求已排队；`FILE LOAD SUCCESS` 表示文件读取成功；`ALIGNMENT SUCCESS` 才表示几何对齐成功。回放会自动推进阶段，通常只输入 `status`、`pose` 或 `quit`。主动退出的 `REPLAY_STOPPED_BY_USER` 不是完整测试通过。

实机无 GUI 模式遇到标准输入 EOF 会退出，因此不能用 `</dev/null` 启动后期待它一直运行。GUI 和自动回放可在输入关闭后继续运行。

## 3. 三类配置与编译

| 文件 | 控制内容 |
|---|---|
| `config/recovery_local.conf` | `DISPLAY_MODE=local`，本机 GUI |
| `config/recovery_headless.conf` | `DISPLAY_MODE=headless`，无 GUI |
| `config/recovery_ssh_windows.conf` | `DISPLAY_MODE=ssh-x11`，SSH 图形转发 |
| `Examples/Stereo-Inertial/RealSense_D435i.yaml` | 一键**实机**的相机/IMU 标定及初始化参数 |
| `recordings/d435i_recovery_verified_20260918_1654/settings.yaml` | 一键**回放**的相机/IMU 配置，独立于实机 YAML |
| `config/px4_vio.yaml` | PX4 安装旋转、发布条件、噪声与通信设置 |

显示配置只接受以下键，注释须单独占一行：

| 键 | 可选值 | 作用 |
|---|---|---|
| `DISPLAY_MODE` | `local` / `headless` / `ssh-x11` | 显示模式 |
| `TERMINAL_CONTROL` | `on` / `off` | 终端控制，三个仓库样例均为 on |
| `SOFTWARE_RENDERING` | `0` / `1` | 只在 ssh-x11 模式设置软件渲染；样例为 1 |

VIBA 配置写在实际使用的相机 YAML 中，修改后重启生效，无需重新编译：

```yaml
IMU.fastInit: 0
IMU.VIBA1Time: 5.0
IMU.VIBA2Time: 15.0
```

`fastInit=0` 保留加速度变化检查，`1` 跳过这一项检查。两段时间均为同一个 `mTinit` 的累计触发阈值，要求 `0 < VIBA1Time < VIBA2Time`，不是两段分别等待，也不是优化运行时长。现有脚本测量的是初始化完成及后续匹配，**尚未实现独立的收敛判定测试**。

Combat 本机既有 `build_sync` 启用测试、PX4 为 OFF，现有离线测试不依赖 ROS。基础配置可用 `cmake -S . -B "$BUILD_DIR" -DORB_SLAM3_BUILD_TESTS=ON -DORB_SLAM3_WITH_PX4=OFF`；依赖路径按本机实际安装补充。

以下为**已安装 RealSense、ROS 和匹配 px4_msgs 后，选择启用 PX4** 的构建示例，不表示这些依赖已安装：

```bash
source /opt/ros/humble/setup.bash
source $HOME/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
cmake -S . -B "$BUILD_DIR" -DORB_SLAM3_BUILD_TESTS=ON -DORB_SLAM3_WITH_PX4=ON
cmake --build "$BUILD_DIR" --target \
  stereo_inertial_realsense_D435i d435i_recovery_replay d435i_capture_slam \
  stereo_inertial_save_replay imu_settings_regression atlas_save_regression \
  recovery_report_regression recovery_console_regression \
  px4_vio_regression px4_bridge_fixture -j2
```

明确不需要 PX4/ROS 时可配置 `-DORB_SLAM3_WITH_PX4=OFF`，并从目标列表去掉 `px4_bridge_fixture`。`px4_vio_regression` 仍可构建。RealSense 实机/录制目标要求找到 librealsense。

## 4. 自定义回放和直接实机入口

回放程序的位置参数：

```text
build/tests/d435i_recovery_replay VOCABULARY SETTINGS EXPORT_DIRECTORY OUTPUT_DIRECTORY [--px4-config FILE]
```

`EXPORT_DIRECTORY` 要包含 `stereo.csv`、`gyro.csv`、`accel.csv` 和本次测试的 `reset_at.txt`，图像路径按 CSV 定位。当前程序阶段日程针对现有两段录制，不是适用于任意 bag 的通用播放器；换录制时需核对阶段时刻。

完整帧、无 GUI、指定 YAML 并保存初始化计时：

```bash
replay_output=$(mktemp -d $ORB_ROOT/recordings/test_runs/manual_replay_XXXXXX)
env -u ORB_REPLAY_VIEWER -u ORB_REPLAY_UNPACED -u ORB_REPLAY_CONSOLE \
  ORB_REPLAY_STRIDE=1 ORB_INIT_METRICS="$replay_output/initialization.csv" \
  "$BUILD_DIR/tests/d435i_recovery_replay" \
  Vocabulary/ORBvoc.txt \
  recordings/d435i_recovery_verified_20260918_1654/settings.yaml \
  recordings/d435i_recovery_verified_20260918_1654/combined \
  "$replay_output" > "$replay_output/run.log" 2>&1
```

将第二个位置参数替换为自己的相机 YAML 即可测试不同配置。保留匹配该录制的标定，通常只改初始化项。

| 环境变量 | 直接运行回放时的默认值 | 作用 |
|---|---|---|
| `ORB_REPLAY_STRIDE` | `1` | 每 N 对图像处理一对；期间全部 IMU 保留。1=全帧，2≈15 Hz 输入 |
| `ORB_REPLAY_VIEWER` | 未设置，关闭 | 存在即启用 GUI，例如 `ORB_REPLAY_VIEWER=1` |
| `ORB_REPLAY_CONSOLE` | 未设置，关闭 | 存在即启用终端输入 |
| `ORB_REPLAY_UNPACED` | 未设置，按传感器帧间隔等待 | 存在即取消等待，测试吞吐 |
| `ORB_INIT_METRICS` | 未设置，不写 | 初始化诊断 CSV 的路径，建议绝对路径；也可用于实机 |

三个开关变量按“是否存在”判断，设成 `0` 仍是开启；关闭要 `unset` 或 `env -u`。一键回放会强制设 `ORB_REPLAY_STRIDE=2` 并按显示/终端选项设置对应变量，外部设 stride=1 不会覆盖它；它未清除继承的 `ORB_REPLAY_UNPACED`，正常限速回放前应确保该变量未设置。

直接实机入口支持不同 YAML：

```bash
"$BUILD_DIR/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i" \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml \
  d435i_recovery --headless --console
```

参数为 `VOCABULARY SETTINGS [trajectory_name] [--headless|--gui] [--console|--no-console] [--px4-config FILE]`，另支持 `--no-gui` 和 `--help`。`d435i_recovery` 是名称，不是数据集路径。直接启动不提供一键脚本的独立运行目录、日志 tee、互斥锁和 MVS 动态库过滤；日常实机优先使用一键入口。

## 5. 初始化时间与帧率基准

默认比较 5/15、3/10、2/5，每组 3 次，顺序交错运行，共 9 次。固定使用现有录制，每次强制 `IMU.fastInit=0`，独立生成 settings.yaml。不会修改实机 YAML。

```bash
benchmark_output=$(mktemp -d $ORB_ROOT/recordings/test_runs/benchmark_XXXXXX)
python3 tests/benchmark_recovery.py "$benchmark_output" \
  --profiles 5:15,3:10,2:5 --repeats 3 --stride 1
python3 tests/report_recovery_benchmark.py "$benchmark_output"
```

| 参数 | 默认 | 含义 |
|---|---|---|
| 位置参数 `output` | 必填 | 输出根目录；使用新目录，已有同名试验子目录会拒绝覆盖 |
| `--profiles` | `5:15,3:10,2:5` | `VIBA1:VIBA2`，多组逗号分隔，单位秒 |
| `--repeats` | `3` | 每组重复次数，至少 1 |
| `--stride` | `1` | 图像步长；测完整输入帧率用 1 |
| `--viewer` | 关闭 | 打开本机 GUI，需要可用 DISPLAY |
| `--unpaced` | 关闭 | 取消回放等待；吞吐试验会改变异步建图调度 |
| `--timeout` | `300` | 每次子进程超时秒数；超时记录退出码 124 |
| `-h` / `--help` | — | 查看参数 |

单组 GUI、无等待吞吐，分别使用新目录：

```bash
gui_benchmark=$(mktemp -d $ORB_ROOT/recordings/test_runs/benchmark_gui_XXXXXX)
env PANGOLIN_WINDOW_URI=x11:// QT_QPA_PLATFORM=xcb EGL_PLATFORM=x11 \
  python3 tests/benchmark_recovery.py "$gui_benchmark" \
  --profiles 5:15 --repeats 1 --stride 1 --viewer
python3 tests/report_recovery_benchmark.py "$gui_benchmark"

throughput_benchmark=$(mktemp -d $ORB_ROOT/recordings/test_runs/benchmark_unpaced_XXXXXX)
python3 tests/benchmark_recovery.py "$throughput_benchmark" \
  --profiles 5:15 --repeats 1 --stride 1 --unpaced
python3 tests/report_recovery_benchmark.py "$throughput_benchmark"
```

报告脚本仅接受一个输出目录位置参数，读取其中 `results.json`，生成 `REPORT.md`、`aggregate.csv` 和 `aggregate.json`，不重新运行 SLAM。它没有 `--help` 等选项。

`benchmark_recovery.py` 保存失败/超时试验，但批量脚本正常结束不保证每次回放通过；检查 `results.json` 中每项的 `pass`。实际 FPS 包含 PNG 读取、跟踪、阶段操作和回放等待，排除词典加载及关闭；TrackStereo 耗时不是整机帧率。

Avi 2026-10-30 的初始化/帧率汇总位于其本机 `recordings/test_runs/replay_benchmark_20261030/REPORT.md`；该数据未随本仓库分发。历史图表可用以下命令重新生成（仅处理该固定目录中的原始数据，不运行 SLAM；需要 numpy、matplotlib）：

```bash
python3 recordings/test_runs/replay_benchmark_20261030/make_report.py
```

## 6. 回归测试与合成存图回放

Combat 分支的 PX4 OFF 构建注册 **7 项**；PX4 ON 且消息依赖齐全时再增加 DDS 项。新增的 `recovery_launcher_paths` 和 `featureless_imu_regression` 检查本地启动路径和无特征帧修复。离线 7 项不需要相机或 ROS 运行环境；DDS 项需要先加载第 3 节 ROS 环境。以实际 `ctest --test-dir "$BUILD_DIR" -N` 列表为准。

```bash
ctest --test-dir "$BUILD_DIR" -N
ctest --test-dir "$BUILD_DIR" --output-on-failure
# 只跑基础 4 项
ctest --test-dir "$BUILD_DIR" --output-on-failure \
  -R '^(imu_settings_regression|atlas_save_regression|recovery_report_regression|recovery_console_regression)$'
# 单项，替换名称即可
ctest --test-dir "$BUILD_DIR" --output-on-failure -R '^imu_settings_regression$'
```

| CTest 名称 | 检查内容 | 若直接调用可执行文件，所需位置参数 |
|---|---|---|
| `recovery_launcher_paths` | 启动脚本路径、参数与只读检查 | 由 CTest 调用 Python |
| `featureless_imu_regression` | 无特征惯性帧提前返回回归 | 由 CTest 提供参数 |
| `imu_settings_regression` | 新旧配置读取、默认/非法参数、实际传参 | 仓库绝对路径 |
| `atlas_save_regression` | 存档读写、过期观测、迭代器失效、空地图 | 无 |
| `recovery_report_regression` | 坐标组合、距离、读档评分及非法数据 | 无 |
| `recovery_console_regression` | 非阻塞终端、错序指令、EOF/退出 | 词典路径、相机 YAML 路径 |
| `px4_vio_regression` | 坐标转换、时钟与发布条件；无需 ROS | 仓库绝对路径 |
| `px4_bridge_ros_regression` | 实际 DDS 传输、停发/恢复、重置计数 | 由 Python 驱动 `px4_bridge_fixture`，见第 8 节 |

C++ 测试二进制在 `$BUILD_DIR/tests/`，启动路径测试由 Python 驱动。CTest 已提供参数；以上测试不覆盖真实图像完整恢复。合成双目＋IMU 存图测试需另外执行：

```bash
save_test_output=$(mktemp -d $ORB_ROOT/recordings/test_runs/synthetic_save_XXXXXX)
"$BUILD_DIR/tests/stereo_inertial_save_replay" Vocabulary/ORBvoc.txt "$save_test_output" \
  > "$save_test_output/run.log" 2>&1
```

仅两个位置参数：词典、输出目录。程序自行生成双目/IMU 与 YAML，无 GUI、无需相机；验证两段 VIBA 后 5 次保存、失败重试、继续跟踪及反序列化。成功日志包含 `PASS: five online saves after VIBA 2`。它没有注册为 CTest，`ctest` 不会自动执行。

## 7. 真正录制数据、导出与拼接

录制程序有 GUI，连接 D435i 后立即开始录制并运行 SLAM，需要人在场提供两次运动激励。它根据真实 VIBA 2 和存图结果自动推进，不是按固定总时长结束；第二次存图后继续保留约 10 秒重叠场景。不执行读档匹配。

```bash
capture_output=$(mktemp -d $ORB_ROOT/recordings/d435i_capture_XXXXXX)
env -u CAPTURE_SECOND_ONLY "$BUILD_DIR/tests/d435i_capture_slam" \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml \
  "$capture_output" > "$capture_output/run.log" 2>&1
```

| 参数 / 控制 | 含义 |
|---|---|
| 位置参数 1/2/3 | 词典 / 相机 YAML / 输出目录 |
| `CAPTURE_SECOND_ONLY=1` | 只录制一段“第二次初始化”，完成后保存 `second.osa`；用于历史补录，不是完整两阶段测试 |
| 不设置 `CAPTURE_SECOND_ONLY` | 第一次 VIBA2→存 first.osa→全量 Reset→第二次 VIBA2→存 second.osa→额外 10 秒 |
| 在输出目录创建 `STOP` | 下次处理帧时提前停止，记录 `CAPTURE_INCOMPLETE`；相机完全停帧时不会立刻处理该文件 |

输出目录不能已有 `session.db3`。该程序不支持 `--headless`、`--replay` 或 `--px4-config`。环境变量同样按存在判断，`CAPTURE_SECOND_ONLY=0` 仍会开启补录模式。成功以 `CAPTURE_COMPLETE` 日志为准。

提前停止示例：在另一终端执行 `touch /实际录制输出目录/STOP`。产物为 `session.db3`、`slam_events.csv`、地图和阶段标记文件；原始 DB3 能再次导出，不要把它当成 `.osa` 地图。

DB3 导出（ROS 2 Humble 消息库、Python OpenCV/numpy）：

```bash
source /opt/ros/humble/setup.bash
python3 tests/d435i_export_recording.py /实际路径/session.db3 /新的路径/export
```

只有两个位置参数，无选项：源 DB3、导出目录。读取源库不修改，生成左右目 PNG、`stereo.csv`、`gyro.csv`、`accel.csv` 和 `export_summary.json`，使用元数据传感器时间。格式针对本机已录制的 RealSense ROS2 DB3，不支持任意 ROS bag。导出可能覆盖目标同名文件，应使用新目录。

两次补录拼接工具：

```bash
python3 tests/prepare_d435i_recovery.py /两段录制的根目录
```

唯一位置参数为根目录，要求以下结构：

```text
根目录/
  export/{stereo.csv,gyro.csv,accel.csv,left/,right/}
  slam_events.csv                          # 包含 first_saved
  second_capture/export/{同上}
  second_capture/slam_events.csv           # 包含 second_saved
```

生成或覆盖 `combined/` 内 CSV、`events.json`、`reset_at.txt`，不改原始录制/导出。**它只处理这种“两次补录”结构**，不会自动拆分新录制程序产生的单个完整 DB3。已有 verified 数据已准备好，日常回放无需再次导出或拼接。

## 8. PX4 通信、DDS 与仿真测试

本节列出当前仓库额外存在的 PX4 测试入口。部署坐标和飞控设置见 [PX4_VIO.md](PX4_VIO.md)，历史验证范围见 [PX4_VALIDATION.md](PX4_VALIDATION.md)。相机测试与桥接测试是不同项目；合成 DDS / SITL 通过不证明 VIO 精度。

### 依赖与安装脚本参数

```bash
python3 scripts/setup_px4_dependencies.py --messages --agent
```

| 脚本 | 参数 | 默认 / 含义 |
|---|---|---|
| `setup_px4_dependencies.py` | `--messages` | 构建固定版本 px4_msgs，至少选择此项或 --agent |
| 同上 | `--agent` | 构建 Agent 2.4.3 及其私有 DDS 依赖 |
| 同上 | `--root DIR` | 默认 `$HOME/.cache/orbslam3-px4` |
| 同上 | `--jobs N` | 默认 3 |
| `setup_px4_sitl.py` | `--root DIR` | 默认同上；下载固定 PX4 源码并构建 SIH nolockstep |
| 同上 | `--jobs N` | 默认 4 |

这些 argparse 脚本以及 `run_px4_agent.py`、`px4_sitl_regression.py` 均支持 `-h` / `--help`。安装脚本会下载和编译，仅缺依赖时运行。其余命令前，在每个新终端先加载：

```bash
source /opt/ros/humble/setup.bash
source $HOME/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
```

### 实机 USB Agent 与 SLAM

```bash
# 终端 1：替换 DEVICE_ID 为实际 /dev/serial/by-id/ 中的飞控名称
python3 scripts/run_px4_agent.py --device /dev/serial/by-id/DEVICE_ID --check
python3 scripts/run_px4_agent.py --device /dev/serial/by-id/DEVICE_ID
# 终端 2
./run_recovery_test.sh --headless --px4-config config/px4_vio.yaml
```

Agent 参数：`--device PATH` 必填；`--agent PATH` 可选，默认 `$HOME/.cache/orbslam3-px4/agent_install/bin/MicroXRCEAgent`；`--check` 只检查可执行文件并打印 USB 是否存在，不连接串口。正常启动固定 serial/921600，等待设备、断开后重连，Ctrl+C 退出。不要把 Agent 私有 lib 目录加入 SLAM 的动态库环境。

桥接 YAML 的当前默认值：

| 键 | 默认 | 含义 |
|---|---|---|
| `extrinsics_valid` | `0` | 未确认安装外参时只回读状态，不发定位 |
| `R_BI` | 3×3 单位阵占位 | D435i IMU→飞机 FRD 旋转，实机须用实际安装标定 |
| `position_std_m` | `0.1` | 位置标准差初值，米 |
| `velocity_std_m_s` | `0.2` | 速度标准差初值，米/秒 |
| `orientation_std_rad` | `0.08726646` | 姿态标准差初值，弧度 |
| `stable_seconds` | `1.0` | 连续有效跟踪等待时间 |
| `max_age_seconds` | `0.2` | 最大观测时龄 |
| `domain_id` / `topic_prefix` | `0` / 空字符串 | 实机 DDS domain / 命名空间 |
| `log_path` | `px4_telemetry.csv` | 相对于运行目录的通信日志路径 |

### 独立 DDS 回归、带通信的录制回放

```bash
ctest --test-dir "$BUILD_DIR" --output-on-failure -R '^px4_bridge_ros_regression$'
# 或直接调用：唯一位置参数是 fixture 可执行文件
python3 tests/px4_bridge_ros_regression.py "$BUILD_DIR/tests/px4_bridge_fixture"
```

普通回归固定 domain 43、`/vio_test`，自动启动合成数据 fixture，无需相机或飞控；成功输出 `PX4_ROS_REGRESSION_PASS`。

带桥接的录制回放需要两个终端。下面创建仅供隔离回放的安装配置；单位阵不是实机标定：

```bash
# 终端 1：持续模拟未解锁飞控并订阅，Ctrl+C 停止
python3 tests/px4_bridge_ros_regression.py --serve /tmp/vio_replay_peer.json
```

```bash
# 终端 2：先 source ROS 环境
px4_replay_config=$(mktemp /tmp/orb-px4-replay-XXXXXX.yaml)
sed 's/^extrinsics_valid: 0$/extrinsics_valid: 1/' config/px4_vio.yaml > "$px4_replay_config"
./run_recovery_test.sh --replay --headless --px4-config "$px4_replay_config"
```

`--serve LOG` 是唯一服务模式参数，写 JSON 计数、重置计数和非法值统计；它不自动结束。该 Python 脚本没有 argparse 帮助。回放程序强制 domain 42、`/vio_test`，并重映射采样时间用于数据路径验证；不会按配置 domain 0 向实机发回放定位。原始配置保持 `extrinsics_valid=0` 会收不到定位，这是正常门控。

### PX4 SIH 仿真

首次准备仿真环境：

```bash
python3 -m venv --system-site-packages $HOME/.cache/orbslam3-px4/venv
source $HOME/.cache/orbslam3-px4/venv/bin/activate
pip install ninja kconfiglib 'empy>=3.3,<4' pyros-genmsg pyulog pymavlink jinja2 pyyaml jsonschema packaging
python scripts/setup_px4_sitl.py --jobs 4
```

加载 ROS 与上述 venv 后运行，输出目录必须尚不存在：

```bash
sitl_parent=$(mktemp -d $ORB_ROOT/recordings/test_runs/px4_sitl_XXXXXX)
python tests/px4_sitl_regression.py \
  --fixture "$BUILD_DIR/tests/px4_bridge_fixture" \
  --px4-build $HOME/.cache/orbslam3-px4/PX4-Autopilot/build/px4_sitl_nolockstep \
  --output "$sitl_parent/result"
```

| 参数 | 含义 |
|---|---|
| `--fixture FILE` | 必填，`px4_bridge_fixture` 可执行文件 |
| `--output DIR` | 必填，新目录，保存 Agent/PX4/桥接日志、JSON 和 ULog |
| `--px4-build DIR` | 默认上述 `px4_sitl_nolockstep` 构建目录 |
| `--agent FILE` | 默认上述缓存中的 MicroXRCEAgent |

固定 instance 17、domain 44、`/vio_sitl`、UDP 18888，使用合成观测测试真实 PX4 EKF，脚本管理并关闭自己启动的进程。它没有相机、GUI、stride 或 VIBA 参数。

`$BUILD_DIR/tests/px4_bridge_fixture [--sitl]` 是上述测试驱动的合成数据源：默认 domain 43；`--sitl` 改为 44。它的终端指令为 `lost`、`resume`、`reset`、`clock`、`stale`、`velocity`、`stationary`、`quit`，分别模拟跟踪丢失、恢复、参考重置、时钟变化、旧观测、运动、静止及退出；这些不是 SLAM 主程序的阶段指令。

## 9. 输出位置与结果判定

| 入口 | 输出 / 通过标志 |
|---|---|
| 一键实机/回放 | `recordings/test_runs/<时间>_<live或replay>_<随机串>/`；`launch.txt`、`run.log`、配置副本 |
| 实机首次存图 | 本次目录中的 `orbslam3_first_map.osa` |
| 真实回放 | `first.osa`、`trace.csv`、`navigation.csv`、`timing.csv`、`timing_metadata.csv`；匹配成功才有 `recovery_report.csv` |
| 回放通过 | 退出码 0 且 `RECOVERY_REPLAY_PASS`；导航检查同时输出 `NAVIGATION_FRAME_PASS` |
| 初始化计时 | 显式设置 `ORB_INIT_METRICS` 后的 CSV；基准脚本自动启用 |
| 基准 | 每次子目录的日志/计时/配置，根目录 `results.json`、`environment.json`；运行报告脚本后生成汇总 |
| 录制 | 原始 `session.db3`、事件和地图；`CAPTURE_COMPLETE` 表示录制条件完成，不表示读档匹配成功 |
| CTest | 测试返回值和失败输出；`ctest -N` 仅列目录，不算执行通过 |
| PX4 | `px4_telemetry.csv`、DDS peer JSON，或 SITL 指定目录的 `summary.json`、日志/ULog |

目前没有单独的“证明 2/5 已收敛”脚本；VIBA 完成、读档评分与点对 RMSE 都不能直接证明绝对定位精度。

相关详解：[初始化配置](IMU_INITIALIZATION_CONFIG.md)、[回放基准](REPLAY_BENCHMARK.md)、[SSH/终端](REMOTE_AND_CONSOLE.md)、[导航坐标系](NAVIGATION_FRAME.md)、[PX4](PX4_VIO.md)。
