# Nano 硬件与软件栈

## 平台

| 项目 | 配置 |
|---|---|
| 板卡 | Jetson Orin Nano Super，6× Cortex-A78AE |
| 功耗模式 | 25 W |
| 内存 / 存储 | 7.4 GiB / 233 GB |
| 系统 | Ubuntu 22.04.5 LTS / aarch64 |
| 内核 | 5.15.148-tegra |
| JetPack | 6.2（R36.4.7） |
| ROS 2 | Humble |

性能记录需同时注明功耗模式与实际频率。已有处理对照使用 CPU 上限 1.344 GHz，不能按标称最高主频换算帧率。

## 相机与图像

| 项目 | 配置 |
|---|---|
| 相机 | DECXIN（德创信）双目 UVC 模组 |
| USB | `1bcf:2d52`，Sunplus 控制器，USB 3 |
| 输入 | MJPEG 2720×960 @ 30 fps |
| 帧布局 | 左侧 160 px 填充，随后为左右各 1280×960 |
| 校正尺寸 | 每目 1280×960 |
| 常驻处理尺寸 | 每目 **960×720** |
| 特征配置 | **nFeatures 1000**，8 层金字塔，比例 1.2，FAST 20/7 |
| 基线 | 63.175 mm |
| 安装 | 镜头向前，模组顶边向上 |

裁剪和双目标定位于 [DECXIN_rectify.yaml](../../Examples/Stereo-Inertial/DECXIN_rectify.yaml)。
[DECXIN_Camera.yaml](../../Examples/Stereo-Inertial/DECXIN_Camera.yaml) 保存全尺寸校正内参，
[DECXIN_Camera_Light.yaml](../../Examples/Stereo-Inertial/DECXIN_Camera_Light.yaml) 指定常驻处理尺寸。
运行时内参与 `bf` 随尺寸同比例缩放，物理基线保持不变。
参数含义及对照见 [处理配置](../../docs/RESOLUTION_PROFILES.md)。

DECXIN 使用 UVC 图像与 PX4 原始 IMU，不依赖 D435i 内置 IMU、hidraw 或 librealsense。
仓库保留的 `vio-camera-prepare` 只处理旧 D435i 的 `8086:0b3a`。

## IMU、外参与时间

| 项目 | 当前定义 |
|---|---|
| IMU 来源 | PX4 `SensorCombined`，约 100 Hz |
| 旋转 | 校正后左目光学坐标 → PX4 body FRD，包含校正旋转 R1 |
| 尺量平移 | 左目光心相对飞控 IMU：前 150 mm、右 20.5 mm、下 49.5 mm |
| 图像时间偏移 | 图像读出后主机时间戳减约 44.534 ms |
| 标定采集条件 | MJPEG 2720×960 @ 30 fps、单 V4L2 缓冲、读出后软件时间戳 |

惯性模式强制保留上述采集时序。服务文件中的双缓冲、取流流水线设置仅供纯双目路径使用。
外参和时间配置见 [decxin_px4_imu.yaml](../../config/decxin_px4_imu.yaml)，验证记录见 [标定报告](../../docs/DECXIN_EXTRINSICS_20261110.md)。

当前桥接输出位于 PX4 IMU 中心，相机杆臂已计入 `IMU.T_b_c1`。
该入口对应的 `EKF2_EV_POS_X/Y/Z` 应为 0；实际飞控参数仍需读取确认，不能重复套用旧 D435i 杆臂。
说明见 [飞控接入](flight-controller.md)。

## 飞控链路

| 项目 | 配置 |
|---|---|
| 飞控固件 | PX4 1.17 |
| 物理链路 | CP2102 USB-UART（`10c4:ea60`）→ `/dev/ttyUSB0` |
| 波特率 | 921600 |
| 协议 | uXRCE-DDS |
| Agent | MicroXRCEAgent 2.4.3 |
| DDS | domain 0，空 topic prefix |

CP2102 曾出现 USB `-32`、断连和重新枚举，造成真实 IMU 缺口。
短时约 100 Hz 接收正常不代表长期连续性已通过；故障和验收状态见 [五米任务流程](../../docs/DECXIN_FIVE_METRE_FLIGHT.md)。

## 软件与设备目录

| 路径 | 用途 |
|---|---|
| `~/ORB_SLAM3/` | 本分支源码、构建与单次运行目录 |
| `<构建目录>/Examples/Stereo/stereo_uvc` | DECXIN 采集与定位入口 |
| `~/.cache/orbslam3-px4/agent_install/` | DDS Agent |
| `~/.cache/orbslam3-px4/ros2_ws/` | 与飞控匹配的 px4_msgs |
| `~/orb_dependencies/` | Pangolin 等设备依赖 |
| `~/inrec/` | 连续状态 CSV 与完整性摘要 |
| `~/maps/` | Atlas 地图存档 |
| `~/vio_image_events/` | 按需冻结的图像事件 |

启动器的构建目录由 `BUILD_DIR` 指定，未设置时为仓库内 `build/`。
Pangolin 仍是构建依赖，无 GUI 运行不会取消该依赖。构建步骤见 [构建说明](../../docs/BUILDING.md)。
`~/.cache/orbslam3-px4/` 保存服务运行依赖，不能按普通临时缓存清理。

## 时间与网络

`vio-clock-prepare` 在服务启动时恢复持久化时钟边界。
运行记录同时保留消息、主机和单调时间；系统时间异常时应结合桥接的时间同步状态判断，不能只看文件名。

有线直连用于 SSH 与文件传输，Wi-Fi 为备用。相机走 USB，飞控走串口与 DDS，服务不依赖外网。
