# Combat UAV Navigation · RoboMaster 2027

构建开关、Ninja/ccache、两分支环境及 CI 边界见 [构建说明](docs/BUILDING.md)。

**南京理工大学江阴校区 Combat 战队 2027 赛季无人机导航方案。** 基于 ORB-SLAM3，用 RealSense D435i 的双目红外图像与 IMU 做视觉惯性定位，并通过可选的 ROS 2 / PX4 接口向飞控提供外部里程计。

无人机在赛场上最容易出问题的环节是参考系。一次碰撞、一次姿态突变，或者只是视觉短暂失效，位置估计就会跳掉。飞控把姿态回正之后，导航侧却不知道自己现在在哪、速度是多少、还能不能接着飞。这个仓库处理的是碰撞之后的定位恢复。

围绕这条链路，我们保留了地图保存与加载、重置后重新初始化、跨地图几何对齐，以及统一导航坐标系下的状态接口。目前定位与恢复流程的基础实现、离线回归和合成数据测试已经跑通；**碰撞后自主回正与导航恢复仍是整机联调目标**，尚无本分支的实飞验收结果，当前地图恢复由终端或 GUI 分阶段确认。

## 与上游的差异

上游 [UZ-SLAMLab / ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB-SLAM3) 是一套完整的视觉/视觉惯性 SLAM 库，定位能力本身不需要重做。本分支的改动集中在工程可用性和导航接口上。

| 环节 | 上游 ORB-SLAM3 | 本分支 | 变化 |
|---|---|---|---|
| 构建体系 | C++11/14，手写 Thirdparty 构建脚本 | C++17 + 现代 CMake，统一 `build.sh` 入口 | 24 个数据集示例全部构建通过；96 个编译单元统一 C++17 |
| 输出布局 | 可执行文件散落在各示例目录 | 统一输出到 `build/`，库与示例分离 | 调用路径固定，便于脚本化 |
| 线程退出 | 退出时可能挂起 | 修复线程退出与启动路径 | 可被脚本反复拉起，不必手动 kill |
| 导航状态 | 只返回内部地图到相机的 `Tcw` | `GetNavigationState()` 同一快照返回位姿、速度、重力、参考变化 | 810 个有效快照、736 个含速度快照一致性检查通过 |
| 地图恢复 | 跟踪丢失即重建，无跨会话位置参考 | 保存 Atlas → 重置 → 再建图 → 加载旧图 → 几何对齐 | 合成回放完成 5 次在线保存、读回及重置检查 |
| 飞控接入 | 无 | 可选 PX4 VIO 桥接：FRD 转换、时效与状态检查 | 离线转换与门限检查通过（未接真机） |
| 回归测试 | 无 CTest 体系 | 7/7 CTest | 覆盖参数读取、序列化、变换组合、终端控制、启动路径、无特征帧 |

## 系统概览

```mermaid
flowchart LR
    SENSOR["D435i<br/>双目红外 + IMU"] --> SLAM["ORB-SLAM3<br/>跟踪、建图、惯性初始化"]
    SLAM --> NAV["导航状态<br/>位姿、速度、重力、参考变化"]
    ATLAS["Atlas 存档<br/>旧图加载与几何对齐"] <--> SLAM
    NAV --> BRIDGE["可选 PX4 VIO 桥接<br/>坐标转换、时效与状态检查"]
    BRIDGE --> PX4["PX4 EKF2<br/>外部里程计融合"]
    PX4 --> CONTROL["飞控侧<br/>姿态回正与位置控制"]
```

| 部分 | 职责 |
|---|---|
| 视觉惯性定位 | 处理双目图像与 IMU，估计运动状态，维护局部地图 |
| 地图恢复 | 保存首次地图；重置后建立第二张地图，再与首次地图匹配、对齐 |
| 导航接口 | 在一致的参考系中输出相机/IMU 位姿、速度及有效性信息 |
| PX4 桥接 | 将观测转换为飞控使用的 FRD 表达，检查跟踪、初始化、时钟和链路状态 |
| 飞控 | 完成姿态与位置控制；碰撞后的回正策略需要结合机体和控制器单独验证 |

仓库包含定位、地图恢复与通信代码。碰撞检测、回正控制律和航迹恢复策略需要在整机控制侧继续接入。

## 碰撞后的定位恢复

恢复分两种情况：短时失去图像约束时，优先恢复当前地图内的跟踪；如果原地图已经无法继续使用，则重新初始化，再通过保存的地图找回原有位置参考。

可复现的跨地图流程分四步：

1. **首次建图**：提供足够的运动激励，完成 IMU 初始化及 VIBA 2，保存 Atlas。
2. **重置与再初始化**：清理旧的跟踪状态，建立第二张地图，重新完成惯性初始化。
3. **加载首次地图**：独立读入存档，保留当前地图用于跟踪。
4. **几何对齐**：匹配两张地图，检查几何支持和连续估计的一致性，再把导航输出转换到首次地图的参考系。

文件读入和定位恢复是两个阶段：`FILE LOAD SUCCESS` 表示存档可用，`ALIGNMENT SUCCESS` 表示几何对齐通过。匹配评分用于工程判断，不代表定位正确的概率。

PX4 桥接在跟踪失效、观测过期或时钟异常时停发。地图参考变化后，当前实现要求飞控处于**未解锁状态**，待观测稳定再建立发布参考。因此，地面重置与读档流程尚不能直接用于空中跨地图连续导航。

恢复速度需要结合运动激励、场景纹理、优化耗时和飞控状态实测。默认 `IMU.VIBA1Time=5.0`、`IMU.VIBA2Time=15.0` 是运动门控计时 `mTinit` 的累计触发阈值，**不是"撞击后多少秒恢复"**。参数含义见 [IMU 初始化配置](tests/IMU_INITIALIZATION_CONFIG.md)。

## 运行环境与同步边界

仓库对应两套环境。源码与配置文本可以共用，**运行环境、依赖与构建产物不能互搬**。

| 项 | 试验机 nano（唯一实机） | 本地开发 WSL |
|---|---|---|
| 系统 / 架构 | Ubuntu 22.04.5 LTS / aarch64 / GCC 11.4 | Ubuntu 26.04.1 LTS / x86_64 / GCC 15–16 |
| ROS 2 | Humble | lyrical |
| `px4_msgs` | 2.0.1，`VehicleStatus MESSAGE_VERSION=1` | 两套并存：系统侧 1.17.0（`Status=5`）；桥接用 `~/px4_ws_201` 2.0.1（`Odom=0/Status=1/Position=1`，与 nano 同版） |
| 相机 / 图形 | RealSense D435i，JetPack 6.2 | 无 RealSense SDK，llvmpipe 软件渲染 |
| 运行方式 | `px4-vio.service` 常驻，开机不依赖网络 | 无服务，只做离线编译与回归 |

`src/Px4Bridge.cc` 对消息版本有 `static_assert(Odom=0 && Status=1 && Position=1)`。本地 WSL 系统侧的 `px4_msgs` 1.17.0 不满足该断言，因此另建 `~/px4_ws_201` 工作区，用与 nano 同版的 2.0.1 消息源码在 x86_64 上经 colcon 重新生成；桥接构建与 ROS 回归必须指向该工作区，见[本地 WSL 双消息工作区](#本地-wsl-双消息工作区与桥接构建)。关闭桥接时可用系统侧消息：2026-11-06 `build/` 为 `ORB_SLAM3_WITH_PX4=OFF`、CTest 14/14；`build_local/` 为 `ON`、CTest 17/17，含 `px4_bridge_fixture` 与三个 ROS DDS 回归。**WSL 的编译与测试结果不代表设备端实机构建或闭环行为。**

同步方向约定：

- 允许：`nano → WSL`（源码与配置文本）、`nano → master / nano 分支`、`WSL → master`（仅限逐字节核对后确认的通用或设备侧脚本，提交时说明来源）。
- 需逐次明确授权：`WSL → nano 设备` 任何写入、`master → nano 设备` 部署、把 master 新提交合并进 `nano` 分支，以及把 WSL 的编译产物、依赖配置或 26.04 / lyrical 专用配置带入 nano 侧。
- 内录、`*.bak` 与构建产物不进版本库；Windows 侧 Git 检出为 CRLF，与 Linux 端比对前先剥 `\r`，否则会产生大量假差异。

`vio-clock-prepare` 是设备侧通用校时脚本（含 Jetson RTC 名 `nvvrs-pseq-rtc`），设备 `/usr/local/sbin/`、本地 WSL `system/`、master `system/` 与 `nano` 分支 `nano/sbin/` 四处副本内容一致，从哪个目录取字节不改变它的归属。

## 编译

核心依赖为 Linux、C++17 编译器、CMake ≥ 3.16、OpenCV ≥ 4.4、Eigen3 ≥ 3.1、Pangolin、Boost.Serialization 和 OpenSSL。DBoW2、g2o 与 Sophus 源码随仓库提供；D435i 入口另外需要 RealSense SDK。

```bash
git clone https://github.com/MalzaharGeng/ORB_SLAM3.git
cd ORB_SLAM3

BUILD_JOBS=2 bash ./build.sh \
  -DORB_SLAM3_BUILD_TESTS=ON \
  -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Pangolin 安装在自定义目录时，追加 `-DCMAKE_PREFIX_PATH=/path/to/pangolin`。本机使用的前缀为 `$HOME/local/pangolin-0.9.6`，其他机器按实际安装位置填写。

`BUILD_DIR` 默认是仓库内的 `build/`，`BUILD_JOBS` 默认是 2。核心库和示例均输出到构建目录；词典文本不存在时，脚本会解压仓库中的压缩词典。

```text
build/lib/libORB_SLAM3.so
build/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i
build/tests/d435i_recovery_replay
build/tests/stereo_inertial_save_replay
```

D435i 可执行文件仅在 CMake 找到 RealSense SDK 后生成。无 GUI 模式关闭显示窗口，构建仍依赖 Pangolin。官方单目、双目、RGB-D 及惯性数据集示例继续保留，使用方法见 [上游文档](docs/UPSTREAM_README.md)；调用路径需加上实际构建目录前缀。

### 本地 WSL 双消息工作区与桥接构建

本机 WSL（Ubuntu 26.04.1 / ROS 2 lyrical / GCC 15–16 / Python 3.14）装有两套 `px4_msgs`，角色不同：

| 工作区 | 版本 | `VehicleStatus` | 用途 |
|---|---|---|---|
| `~/px4_ws` | 1.17.0 | 5 | 系统侧，随 lyrical 使用；不满足 `Px4Bridge.cc` 的版本断言 |
| `~/px4_ws_201` | 2.0.1 | 1 | 与 nano 同版，`Odom=0 / Status=1 / Position=1 / Battery=1`；桥接构建与 ROS 回归专用 |

`~/px4_ws_201` 是 colcon 工作区：`src/px4_msgs` 为消息源码，`install/px4_msgs/lib/python3.14/site-packages` 提供 Python 侧消息，`install/px4_msgs/share/px4_msgs/cmake` 提供 CMake 包配置。工作区根目录的 `.orb_px4_revision` 记录来源 revision `86d8239…`，用于核对消息定义出处。

**该工作区是本地环境资产，不在仓库里，换机器必须重建。** 不能直接拷贝 nano 的 `install/`：其中是 aarch64 的 `.so` 与 Python 3.10 包，x86_64 / Python 3.14 下不可用。在目标架构上重新生成：

```bash
mkdir -p ~/px4_ws_201/src
# 取与 nano 同版的 px4_msgs 2.0.1 消息源码，放入 ~/px4_ws_201/src/px4_msgs
source /opt/ros/lyrical/setup.bash
cd ~/px4_ws_201 && colcon build --packages-select px4_msgs
source install/setup.bash
python3 -c "from px4_msgs.msg import VehicleOdometry,VehicleStatus,VehicleLocalPosition; \
print([getattr(m,'MESSAGE_VERSION',0) for m in (VehicleOdometry,VehicleStatus,VehicleLocalPosition)])"
# 期望 [0, 1, 1]；不满足时不要开始编译
```

本机另建两套构建目录，分别验证核心逻辑与完整桥接：

```bash
source /opt/ros/lyrical/setup.bash
source ~/px4_ws_201/install/setup.bash   # 只加载这一个，不要 source ~/px4_ws
cd ~/ORB_SLAM3
ctest --test-dir build       --output-on-failure   # OFF，14 项
ctest --test-dir build_local --output-on-failure   # ON，17 项
```

`build_local/` 的 configure 参数为 `-DCMAKE_BUILD_TYPE=Release -DORB_SLAM3_BUILD_TESTS=ON -DORB_SLAM3_WITH_PX4=ON -Dpx4_msgs_DIR=$HOME/px4_ws_201/install/px4_msgs/share/px4_msgs/cmake`。ROS 回归用 `ROS_DOMAIN_ID=45`、`RUN_SERIAL` 与独立 TIMEOUT 隔离。Python 脚本与 JSON 配置不是编译单元，这类改动重跑 CTest 即可，只有 C++ 源码或 CMake 配置变更才需重新 configure 与编译。

2026-11-06 的 WSL 记录：`build/` 06:15 CTest 14/14；`build_local/` 06:50 编出 `liborbslam3_px4.a`、06:53 编出 `px4_bridge_fixture`、CTest 17/17。WSL 无 RealSense SDK、使用 llvmpipe 软件渲染、无实机，这些结果不能替代 nano 设备端构建与实飞验收。

## 运行与调试

### D435i 实机入口

从仓库根目录启动：

```bash
# 先检查程序、配置和动态库，不连接相机。
./run_recovery_test.sh --headless --check

# 无 GUI 运行，终端控制恢复阶段。
./run_recovery_test.sh --headless

# 本机图形界面；SSH X11 转发使用 --ssh-x11。
./run_recovery_test.sh --gui
```

| 指令 | 操作条件与作用 |
|---|---|
| `first` | 首次 VIBA 2 完成且跟踪正常后，保存地图 |
| `reset` | 保存成功后，全量重置并开始第二次建图 |
| `second` | 第二次 VIBA 2 完成后，检查并确认初始化状态 |
| `load` | 进入 `SECOND_READY` 后，加载首次地图并等待几何对齐 |
| `status` / `pose` | 查看阶段、保存状态和导航位姿 |
| `quit` | 停止采集并关闭 SLAM 工作线程 |

每条推进指令都需要等待对应成功状态，`queued` 只表示请求已排队。相机与 IMU 参数位于 [CModel.yaml](Examples/Stereo-Inertial/CModel.yaml)，显示与终端配置位于 `config/recovery_*.conf`。完整操作见 [终端与 SSH 说明](tests/REMOTE_AND_CONSOLE.md)。

### 离线回放

```bash
RECORDING_DIR=/path/to/recording ./run_recovery_test.sh --replay --headless

# 使用独立构建目录时，构建与运行使用相同的 BUILD_DIR。
BUILD_DIR="$PWD/build_sync" RECORDING_DIR=/path/to/recording \
  ./run_recovery_test.sh --replay --headless
```

录制目录需要包含 `settings.yaml` 和脚本要求的图像、IMU、重置时刻等输入，数据不随仓库分发。回放会记录导航结果、恢复阶段与处理耗时。不同初始化阈值的重复实验见 [回放性能测试](tests/REPLAY_BENCHMARK.md)。

## 导航接口与坐标约定

内部 SLAM 地图、导航世界系和飞控参考系分别承担不同用途。这三者混用是接入控制器时最常见的错误来源，所以接口按用途拆开：

| 接口 / 字段 | 含义 |
|---|---|
| `TrackStereo()` | 保留 ORB-SLAM3 的返回约定：内部地图到相机的 `Tcw` |
| `GetNavigationState()` | 同一快照中的位姿、速度、重力、跟踪状态与参考变化信息；先检查 `valid` |
| `GetCurrentPose()` | 左相机到导航世界系的位姿 |
| `GetCurrentImuPose()` | IMU 到导航世界系的位姿 |
| `velocityWorld` / `gravityWorld` | 导航世界系中的速度与重力；速度另检查 `velocityValid` |
| `referenceEpoch` | 标识导航参考变化，供接入端处理重置和坐标跳变 |

导航世界系取当前地图首个关键帧时刻的 **IMU 坐标系**。完成旧图几何对齐后，输出转换到首次保存地图的初始 IMU 坐标系。相机光心与 IMU 中心存在安装偏移，初始相机位置不必为零；初始 IMU 的轴向也不自动等同于 ENU 或 NED。

变换方向、地图点转换与跨地图组合公式见 [导航坐标系说明](tests/NAVIGATION_FRAME.md)。

## PX4 接入

桥接默认关闭。启用前需要安装兼容的 ROS 2、PX4 1.17 `px4_msgs` 与 Micro XRCE-DDS Agent。现有部署说明基于 Ubuntu 22.04 / ROS 2 Humble，具体版本和环境准备见 [PX4 VIO 接入](tests/PX4_VIO.md)。

在依赖和 ROS 环境已准备好的终端中编译：

```bash
BUILD_JOBS=2 bash ./build.sh \
  -DORB_SLAM3_BUILD_TESTS=ON \
  -DORB_SLAM3_WITH_PX4=ON

# 填写安装外参后启动；Agent 在另一终端运行，步骤见 PX4 接入文档。
./run_recovery_test.sh --headless --px4-config /path/to/px4_vio.yaml
```

以 [config/px4_vio.yaml](config/px4_vio.yaml) 为模板，填写实测的 `R_BI` 后再设置 `extrinsics_valid: 1`。它表示 D435i IMU 到机体 FRD 的旋转；IMU 相对机体重心的平移仅在 PX4 的 `EKF2_EV_POS_X/Y/Z` 中配置，避免重复补偿。

桥接发布位置、速度、姿态与重置计数，并回读飞控和时间同步状态。输出采用重力对齐、任意初始航向的 FRD 参考系；首次建立参考要求未解锁。默认还要求 VIBA 2 完成、速度有效、连续稳定跟踪 1 s，且观测时龄不超过 200 ms。具体门限与停发条件见接入文档，1 s 稳定窗口不代表整条链路的恢复耗时。

## 验证

以下为 **2026-10-30 本分支在 Ubuntu WSL 中的检查结果**。Avi 文档中的历史实机与仿真记录单独保留，未计入本表。2026-11-06 本地 WSL 另有 `build/` 14/14（关闭 PX4 桥接）与 `build_local/` 17/17（开启 PX4 桥接，由 `~/px4_ws_201` 提供与 nano 同版的消息定义）两次 CTest 记录；两次 WSL 结果都不能替代设备端构建与实飞验收，环境差异见[运行环境与同步边界](#运行环境与同步边界)。

| 检查 | 结果 |
|---|---|
| 核心与示例构建 | 核心库、DBoW2/g2o、24 个普通数据集 Examples 构建通过；96 个 C++ 编译单元使用 C++17 |
| CTest | 7/7 通过，覆盖参数读取、Atlas 序列化、变换组合、终端控制、启动路径、无特征帧与离线 PX4 转换 |
| 合成双目惯性回放 | 完成 VIBA 1/2、5 次在线保存、写入失败重试、存档读回及重置状态检查 |
| 导航状态 | 检查 810 个有效快照及 736 个含速度快照的时间、参考系与数值一致性 |
| 合成 RGB-D + Viewer | 40 帧正常运行并退出；仍有 Qt 跨线程计时器警告 |

```bash
ctest --test-dir build --output-on-failure
./build/tests/stereo_inertial_save_replay \
  Vocabulary/ORBvoc.txt /tmp/orb-save-smoke
```

## 适用范围与限制

C++17 / 现代 CMake 适配、构建树输出、线程退出修复是可复现的工程改动；导航状态接口与跨地图流程的变换组合、序列化、终端控制有 CTest 与合成回放兜底。合成数据结果只用于检查实现，不能代替整机性能指标。本次环境缺少 RealSense SDK 和 `px4_msgs`，**未构建** D435i 入口及 ROS/PX4 桥接。真实相机、完整实录跨地图恢复、飞控通信、碰撞回正、实飞精度与恢复时间均需继续验收。

两处容易误读的地方：

- `ALIGNMENT SUCCESS` 与匹配评分是工程判据，不代表定位正确的概率；
- `IMU.VIBA1Time` / `VIBA2Time` 是运动门控累计阈值，不能换算成"撞击后几秒恢复"。

配置、日志位置和完整边界见 [验证报告](docs/AVI_SYNC_2026-10-30.md)。

## 代码来源与维护

定位算法基于 [UZ-SLAMLab / ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB-SLAM3)，官方基准为 `4452a3c`。地图恢复、初始 IMU 导航、D435i 终端控制及 PX4 接入合并自 [Kuroneko-avi / ORB-SLAM3](https://github.com/Kuroneko-avi/ORB-SLAM3)，当前同步至 `b9ba5ec`（含测试命令汇总），保留原始提交和作者信息。

本分支维护 C++17 / 现代 CMake 适配、构建树输出、依赖目标链接、线程退出与启动路径修复，并补充恢复流程的回归检查。ORB-SLAM3 原作者为 Carlos Campos、Richard Elvira、Juan J. Gómez Rodríguez、José M. M. Montiel 与 Juan D. Tardós；功能来源及本地修改见 [修改说明](docs/LOCAL_CHANGES.md)。

```text
include/、src/          SLAM、导航状态与 PX4 桥接
Examples/              传感器入口、数据集示例与相机配置
Examples_old/          上游旧版示例
config/                恢复启动配置与 PX4 参数模板
tests/                 回归测试、回放与性能统计
scripts/               可选 PX4 依赖与 Agent 工具
Thirdparty/            DBoW2、g2o、Sophus
Vocabulary/            ORB 压缩词典
docs/                  接入说明、来源与验证记录
build.sh               统一构建入口
run_recovery_test.sh   实机与回放启动入口
```

后续联调重点是完整实录恢复、飞控侧姿态回正，以及参考变化期间的定位接管策略。记录实验时保留相机与安装外参、初始化参数、飞控版本、跟踪状态、恢复阶段和时间戳，以便比较不同配置的效果。

## 许可证与引用

本仓库沿用 [GNU GPL v3](LICENSE)，第三方组件遵循各自许可证。使用、修改和分发时请保留原作者与第三方版权声明。

学术工作中使用 ORB-SLAM3，请引用原论文：

```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map {SLAM}},
  author={Campos, Carlos and Elvira, Richard and G\'omez, Juan J. and Montiel, Jos\'e M. M. and Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics},
  volume={37},
  number={6},
  pages={1874--1890},
  year={2021}
}
```

测试入口、参数与输出位置见 [测试命令汇总](tests/TEST_COMMANDS.md)。2026-10-31 的文档增量合并与设备同步范围见 [本轮记录](docs/AVI_SYNC_2026-10-31.md)。

## 分支布局

| 分支 | 内容 |
|---|---|
| `master` | 默认分支，源码主线，两套飞行/建图 profile |
| `nano` | 第二长期分支。与 `master` 同源，包含同一份源码快照，另有 `nano/` 存放试验机设备侧部署物与现场文档 |
| `nano-crash-20261103` | 从 `nano` 分出，2026-11-03 首次炸机的遥测、内录与分析 |
| `nano-crash-20261105` | 从 `nano` 分出，2026-11-05 两次飞行的 ULog、内录、综合分析与修复记录 |
| `data/first-crash-20261103`、`data/second-crash-20261105` | 早期独立历史分支，连同对应 tag 一起保留作归档，不再更新 |

`nano` 在工程记录里按第二主线维护：源码改动先落 `master`；同步到 `nano` 分支需逐次确认，设备侧部署另行授权。设备侧部署物改动落在 `nano/` 下。
GitHub 本身只有默认分支这一个角色，这里的主次关系由同源历史与本表约定体现；`nano` 的分支保护规则尚未在仓库设置中启用。
`nano` 的基底早于当前 `master`，差异可用 `git log nano..master` 查看，目前未做合并；`nano/` 下已有设备侧校时脚本的等价副本。
2026-11-05 之前 `nano` 的独立历史由 tag `nano-deploy-20261105` 与 `archive/nano-deploy-20261106` 保留。

两次炸机资料以 `nano` 的直接子分支归档，见上表；旧 `data/*` 分支与 tag 未改动。
子分支名用连字符而非 `nano/` 前缀：Git 中 `refs/heads/nano` 已作为文件存在，
同一路径不能再建目录形式的子引用。
