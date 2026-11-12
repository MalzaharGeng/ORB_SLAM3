# 构建配置与验证边界

`nano` 面向 Ubuntu 22.04 / Humble / aarch64，当前相机为 DECXIN，采集目标为 `stereo_uvc`。常驻服务使用每目 960×720、nFeatures 1000，见 [处理配置](RESOLUTION_PROFILES.md)。`master` 为 x86 / WSL 开发分支；两端分别构建，依赖和二进制不能跨架构搬运。

## 环境与默认值

| 项 | master | nano | 验证条件 |
|---|---|---|---|
| 相机测试配置 | `Examples/Stereo-Inertial/DECXIN_Camera.yaml` | `Examples/Stereo-Inertial/DECXIN_Camera.yaml` | 2026-11-09 起两分支同名同内容 |
| 部署环境 | 本机 WSL Ubuntu 26.04 / ROS 2 Lyrical / x86_64 | Ubuntu 22.04 / ROS 2 Humble / aarch64 | WSL 不代替设备验收 |
| `ORB_SLAM3_NATIVE_ARCH` | OFF | ON | ON 仅供在目标机器本地构建；交叉编译时明确拒绝 |
| CPU 参数 | ON 时 x86 使用 `-march=native` | ON 时 ARM64 使用 `-mcpu=native` | 只作用于 Release/RelWithDebInfo；Nano ARM 编译待设备验证 |
| CMake / C++ | ≥3.16 / C++17 | ≥3.16 / C++17 | 本机使用 CMake 4.2；CI 使用 Ubuntu22 所带版本 |
| 核心依赖 | OpenCV≥4.4、Eigen3≥3.1、Pangolin、Boost.Serialization、OpenSSL、Threads | 同左 | Pangolin 仍为构建依赖 |
| px4_msgs | 需满足 Odom=0、Status=1、Position=1 | 同左 | 构建前加载与该固件兼容的消息环境，不混用系统侧新版定义 |
| CI | Ubuntu22 GCC11 core + Humble/PX4 x86_64 | 同一兼容性矩阵 | 不等同于 Lyrical 本机环境或 Nano 实机 |

## 目标选择

| CMake 选项 | 默认值 | 含义 |
|---|---|---|
| `ORB_SLAM3_WITH_PX4` | OFF | ROS2/PX4 桥接；开后需要 ament、rclcpp、std_msgs、px4_msgs |
| `ORB_SLAM3_BUILD_TESTS` | OFF | CTest、离线回放与必要测试程序 |
| `ORB_SLAM3_BUILD_DATASET_EXAMPLES` | ON | 12 个当前数据集示例 |
| `ORB_SLAM3_BUILD_CAMERA_EXAMPLES` | ON | 生成 `stereo_uvc`；RealSense 入口另需 SDK，与数据集开关独立 |
| `ORB_SLAM3_BUILD_LEGACY_EXAMPLES` | OFF | 12 个历史数据集示例及 SDK 可用时的历史采集程序 |
| `ORB_SLAM3_REQUIRE_REALSENSE` | OFF | ON 时缺少 RealSense SDK 立即失败；DECXIN 不要求该 SDK |
| `ORB_SLAM3_WITH_CUDA_FAST` | OFF | 构建 CUDA FAST 与双目校正后端 |
| `ORB_SLAM3_CUDA_ARCH` | 87 | CUDA 目标架构，当前 Orin 使用 87 |

`feature_profile_benchmark` 不再进入默认 all，仍可显式按目标构建。g2o 生成头文件位于构建目录 `Thirdparty/g2o/g2o/config.h`，不再写源码树。RealSense SDK 只链接采集程序，核心库不再因为检测到 SDK 而绑定它。

## 本机编译

先按本机安装位置加载 Pangolin 与对应 ROS/px4_msgs 环境。ROS/PX4 环境只在启用桥接时需要，不在仓库写死个人工作区。

```bash
# ROS_SETUP / PX4_MSGS_SETUP 指向本机相应 setup.bash。
source "$ROS_SETUP"
source "$PX4_MSGS_SETUP"

# 离线与桥接测试，不编数据集和相机示例。
BUILD_JOBS=2 bash ./build.sh \
  -DORB_SLAM3_WITH_PX4=ON -DORB_SLAM3_BUILD_TESTS=ON \
  -DORB_SLAM3_BUILD_DATASET_EXAMPLES=OFF \
  -DORB_SLAM3_BUILD_CAMERA_EXAMPLES=OFF
ctest --test-dir build --output-on-failure
```

仅核心测试：省略 ROS 加载，把 `ORB_SLAM3_WITH_PX4` 设为 OFF。测试需要词库，`build.sh` 只在缺少文本时解压自带词库。

```bash
# 同一构建目录保留当前生成器；新的目录在 Ninja 可用时优先选 Ninja。
BUILD_DIR=build_debug USE_CCACHE=0 bash ./build.sh \
  -DCMAKE_BUILD_TYPE=Debug -DORB_SLAM3_BUILD_TESTS=ON \
  -DORB_SLAM3_BUILD_DATASET_EXAMPLES=OFF -DORB_SLAM3_BUILD_CAMERA_EXAMPLES=OFF

# 构建配置检查（只 configure，不编译）：目标集合、Debug参数、源码树无生成头写入。
python3 tests/build_configuration_regression.py
```

| 脚本变量 | 默认值 | 处理 |
|---|---|---|
| `BUILD_DIR` | `build/` | 不自动清空旧目录，不在旧目录强制换生成器 |
| `BUILD_JOBS` | 2 | 必须为正整数；Nano 受内存约束，先维持2 |
| `USE_CCACHE` | 1 | 有 ccache 才启用；设0会清除当前配置的 compiler launcher |
| `CMAKE_GENERATOR` / `-G` | 未设置 | 新目录自动选 Ninja；用户显式选择优先 |

主工程、DBoW2 和 g2o 不再把 `-O3` 无条件叠加到 Debug。Release 优化采用 CMake 对应编译器的默认值。未启用 PCH/LTO，也未把编译选项改动表述为 SLAM 运行性能提升。

ROS 依赖在 Humble 优先沿用 `ament_target_dependencies`，Lyrical 缺少该命令时使用包导出的 imported targets。该兼容逻辑位于 `cmake/RosDependencies.cmake`，不再要求本机额外注入 ament 兼容脚本。

## Nano 设备构建约束

切到 `nano` 分支后，在 Nano 本机加载 Humble、对应 px4_msgs 和设备依赖，再执行构建；不要把 WSL 的环境脚本或库路径复制过去。

```bash
BUILD_JOBS=2 bash ./build.sh \
  -DORB_SLAM3_WITH_PX4=ON -DORB_SLAM3_BUILD_TESTS=ON \
  -DORB_SLAM3_BUILD_DATASET_EXAMPLES=OFF \
  -DORB_SLAM3_BUILD_CAMERA_EXAMPLES=ON -DORB_SLAM3_REQUIRE_REALSENSE=OFF \
  -DORB_SLAM3_WITH_CUDA_FAST=ON -DORB_SLAM3_CUDA_ARCH=87
```

DECXIN 目标输出到 `build/Examples/Stereo/stereo_uvc`。CUDA 后端需要设备上的 CUDA 编译器；纯 CPU 构建设为 `ORB_SLAM3_WITH_CUDA_FAST=OFF`，运行时选择 CPU 后端。更新设备构建前先停止相关服务，部署另行授权。下方 2026-11-08 的检查结果保留原实验条件，当前设备状态见 [五米任务流程](DECXIN_FIVE_METRE_FLIGHT.md)。

## CI

`.github/workflows/build.yml` 在 master/nano push、PR 和手动触发时运行两个作业。核心作业不加载 ROS，桥接作业用 Humble＋固定 px4_msgs 提交。Pangolin 固定 v0.9.6 对应提交 `3667bb76…`，消息提交 `86d8239…`；cache 区分 OS、架构、编译器、profile 和构建配置。只上传构建/测试日志，保留7天，不上传内录、相机数据或可部署二进制。

依赖源码固定不等于容器/系统包完全可复现：目前仍使用发行版镜像标签和 apt 软件源。CI 的最终成功状态要看对应提交的 Actions，不把 YAML 解析通过写成 CI 运行通过。

## 已知问题

2026-11-08 本轮本地检查（不含实机）：

| 分支源码 | 执行环境与配置 | 结果 |
|---|---|---|
| master | WSL Ubuntu26.04/x86_64，Lyrical＋px4_msgs2.0.1；数据集/相机/历史示例关，TESTS/PX4开 | 全量测试程序构建完成，CTest 17/17 |
| master | Release最小、Debug、当前示例、历史示例四组配置 | 最小66编译单元、当前数据集12个、历史数据集显式12个；Debug无-O3/native，g2o源码树无生成头写入 |
| nano | 同一本机环境，仅构建转换门控与ROS桥接fixture | 三项C++＋两项ROS回归5/5；不是aarch64设备验证 |
| 两分支共享构建文件 | 逐文件SHA256 | 共用源码一致；仅保留native默认值与相机测试路径差异 |

首次跨工作区并发ROS测试发生domain43串扰，失败日志保留在本机检查目录；停止并发后按分支顺序重测通过。未通过删除断言、放宽运行门控或修改SLAM算法消除失败。

| 现象 | 原因 | 当前处理 |
|---|---|---|
| 无 SDK 时找不到 D435i target | RealSense 目标依赖 SDK | DECXIN 使用 `stereo_uvc`，不要求 RealSense SDK |
| headless 仍需 Pangolin | Viewer/MapDrawer 尚在核心库 | 保留依赖，不宣称已有纯无图形核心 |
| 历史 ROS1 脚本不可用于 ROS2 | 旧 rosbuild 目录已迁走，ROS接口不同 | master 的 build_ros.sh 返回2并给出新入口；nano 保持无该脚本 |
| 不同环境共享 build 目录会出错 | 编译器、消息Python ABI、架构或生成器不同 | 新建独立构建目录，不搬 cache/产物 |
| 两份工作区同时跑 ROS 回归会互相收到模拟消息 | `RUN_SERIAL` 只管一份 CTest 进程；已有回归固定使用 domain43/45 | 本机跨工作区顺序运行；CI作业分开容器，不并发共用本机测试域 |
| 编译仍有既有告警 | 上游模板及当前SLAM代码有未使用变量等告警 | 不在构建调整中改运行算法，告警不等于测试失败 |

## 待办

- 2026-11-08 构建调整的 Nano 全量原生验收当时未执行；后续 DECXIN 增量构建与服务记录见 [接入记录](DECXIN_VIO_INTEGRATION_20261110.md) 和 [五米任务流程](DECXIN_FIVE_METRE_FLIGHT.md)。
- 真实相机/实飞、故障注入、断链与运行性能基准；构建回归不证明这些项目通过。
- 独立拆分 Viewer 与定位转换库，降低纯门控测试的依赖；本轮未实施。
- LTO/PCH 与编译内存/运行耗时对照；没有测量前不启用。
