# 硬件与软件栈

## 板卡

| 项 | 值 |
|---|---|
| 机型 | Jetson Orin Nano Engineering Reference Developer Kit Super |
| CPU | 6× Cortex-A78AE，最高 1728 MHz |
| 功耗模式 | `nvpmodel` 1，25 W |
| 内存 | 7.4 GiB，空闲时约 5.5 GiB 可用 |
| 存储 | 233 GB，系统占用约 13% |
| 系统 | Ubuntu 22.04.5 LTS aarch64 |
| 内核 | 5.15.148-tegra |
| JetPack | R36.4.7（JetPack 6.2） |

空载时 CPU 在 1190–1344 MHz 之间浮动，属正常 DVFS；25 W 功耗墙下能否长时间维持
1728 MHz 没有实测。空载温度 47 °C（TJ 48 °C），远低于 90 °C 阈值，不构成热降频。

## 相机

| 项 | 值 |
|---|---|
| 型号 | Intel RealSense D435i，USB `8086:0b3a` |
| 设备节点 | `/dev/video4`、`/dev/video5`，枚举完整时 video ≥ 6、hidraw ≥ 1 |
| 使用模式 | 双目 IR（GREY 8 bit），不是 RGB |
| 分辨率 / 帧率 | 640×480 @ 15 fps |
| 安装 | 水平安装，镜头朝机头 |
| 内参 | `fx = fy = 386.125`，`cx = 321.161`，`cy = 235.601`，基线 `0.0500186` m |

上表是 master 里 [Examples/Stereo-Inertial/RealSense_D435i.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/RealSense_D435i.yaml) 锁的值。实机入口
启动时会把它和 SDK 活动 profile 报出的内参、基线、相机到 IMU 外参比一遍
（0.002 px / 1 μm / 1e-5），不一致直接拒绝启动，并在运行目录写
`camera_calibration.json`。所以这些数字不需要在设备上再抄一份，改只改 master。

相机对 USB 枚举敏感：librealsense 退出后若把 usbfs 接口据着不放，video 会掉到 2、
hidraw 消失，此时干等不会恢复，只能重新枚举。这也是 `vio-camera-prepare` 存在的原因。

## 相机到 IMU

D435i 内置 IMU 200 Hz，与图像同一时钟域，SLAM 用双目惯性模式。

| 变换 | 位置 | 含义 |
|---|---|---|
| `IMU.T_b_c1` | master 的 [RealSense_D435i.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/RealSense_D435i.yaml) | 左目到 D435i 内部 IMU，由 SDK 活动 profile 给出 |
| `R_BI` | master 的 [config/px4_vio_local.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio_local.yaml) | D435i IMU 到机体 FRD，实测安装朝向 |
| `EKF2_EV_POS_X/Y/Z` | 飞控 | 那个 IMU 到机体重心的杆臂 |

三者不要混用。桥接输出的是 IMU 中心位姿，杆臂只在飞控补偿；两边都补等于补两次。
杆臂当前三项都是 0，尚未补偿，见 [flight-controller.md](flight-controller.md)。

## 飞控链路

| 项 | 值 |
|---|---|
| 飞控固件 | PX4 1.17 |
| 物理链路 | Silicon Labs CP2102 USB-UART（`10c4:ea60`）→ `/dev/ttyUSB0` |
| 波特率 | 921600 |
| 协议 | uXRCE-DDS（`SERIAL1_BAUD=921600`、`UXRCE_DDS_CFG=102`） |
| Agent | MicroXRCEAgent 2.4.3 |
| DDS | domain 0，`topic_prefix` 为空 |

电台接飞控 SERIAL2 走 MAVLink 到 QGC，不经过 Nano；Nano 侧的串口只有 CP2102 一个。

## 软件栈

| 组件 | 版本 / 路径 |
|---|---|
| ROS 2 | Humble，`/opt/ros/humble` |
| ORB-SLAM3 | `~/ORB_SLAM3`，构建目录 `build_nano`；源码见 [master](https://github.com/MalzaharGeng/ORB_SLAM3/tree/master) |
| 可执行 | `build_nano/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i` |
| librealsense | 2.56.5，V4L2 后端（`librealsense-2.56.5-v4l`） |
| Pangolin | 0.9.6 |
| px4_msgs | `~/px4_ros2_ws/install/px4_msgs`，另有一份在 `~/.cache/orbslam3-px4/ros2_ws` |
| 依赖根 | `~/orb_dependencies/` |

依赖不装系统包，用 `dpkg-deb` 解包进 sysroot，可执行文件靠 `DT_RPATH` 找库。这样
系统升级不会把依赖换掉，代价是换机器要整目录搬。

librealsense 有两套并存：RSUSB 后端在这块板子上报 `failed to set power state`，IMU
也起不来；V4L2 后端两个问题都没有。切换后 `strace` 里不应再出现 librsusb，验收方式
是 `ldd` 加 `ctest`，见 master 的 [tests/README.md](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/tests/README.md)。

## 目录布局

```
~/ORB_SLAM3/                源码与构建，recordings/test_runs/ 下是每次运行的产物
~/.cache/orbslam3-px4/      agent_install/（MicroXRCEAgent）、ros2_ws/（px4_msgs）
~/orb_dependencies/         解包的依赖 sysroot
~/inrec/                    同步内录 CSV
~/maps/                     Atlas 存档，active 记录下次加载哪一张
~/vio_image_events/         冻结的图像事件
```

`~/.cache/orbslam3-px4/` 是隐藏目录但不可清理：服务启动第一件事就是从这里找 Agent。
断电重装只备份 `~/maps/` 和 `~/inrec/` 就够，其余可从 master 重建。

## 时间基准

板载 RTC 读出 1970，`fake-hwclock` 虽然 enabled + active、数据文件里存着正确时间，
开机仍被内核 `hctosys` 用 RTC 覆盖，系统时间停在 `1970-01-01 08:xx`。

后果是录制文件名全部 `inrec_19700101_*`，桥接的 `sample_clock_invalid` 与
`time_sync_unready` 会一直触发。这条没解决前，涉及绝对时间的判读都不可靠；相对时间
（steady clock）不受影响，遥测和图像缓冲都按后者对齐。

## 网络

有线网口与 PC 同网段直连（192.168.50.0/24），是控制与传文件的主通道；WiFi 只作备用，
不参与控制。服务本身不依赖网络：Agent 走串口，VIO 走 DDS，断网只影响 SSH 和遥测外传。
