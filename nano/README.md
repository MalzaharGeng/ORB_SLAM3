# Nano 设备部署

本目录保存 Jetson Orin Nano 的 systemd 单元、守护脚本、录制工具和遥控器授权任务服务。
源码与相机配置位于仓库根目录，设备路径为 `/home/nano/ORB_SLAM3`。

当前链路为 **DECXIN 双目相机 + PX4 原始 IMU**，常驻处理为每目 **960×720、nFeatures 1000**。
服务使用 `stereo_uvc`、YUYV 亮度直取、GPU 校正与 FAST、CPU 层级并行；惯性模式使用单缓冲和重新验证的 -52.280 ms 时间偏移。
硬件、配置和当前验收状态分别见 [硬件说明](docs/hardware.md)、[处理配置](../docs/RESOLUTION_PROFILES.md) 和 [五米任务流程](../docs/DECXIN_FIVE_METRE_FLIGHT.md)。

## 目录

```text
bin/               定位守护、状态内录、重启门控、地图与图像工具
sbin/              启动校时、旧 D435i USB 恢复工具
systemd/           定位服务与相机恢复权限配置
pilot-navigation/  遥控器授权任务：脚本、配置、测试与服务单元
px4/               历史飞控参数文件
docs/              硬件、守护、录制、飞控与任务说明
docs/archive/      旧 D435i 部署手册和飞控参数记录
verification/      按日期保存的部署及验证证据
```

## 当前配置文件

| 文件 | 用途 |
|---|---|
| [DECXIN_Camera_Light.yaml](../Examples/Stereo-Inertial/DECXIN_Camera_Light.yaml) | 常驻服务：每目 960×720、1000 特征 |
| [DECXIN_Camera.yaml](../Examples/Stereo-Inertial/DECXIN_Camera.yaml) | 全尺寸标定与回退配置 |
| [DECXIN_rectify.yaml](../Examples/Stereo-Inertial/DECXIN_rectify.yaml) | 原始双目标定、裁剪与校正 |
| [decxin_px4_imu.yaml](../config/decxin_px4_imu.yaml) | PX4 原始 IMU、旋转外参与图像时间偏移 |
| [decxin_px4_flight.yaml](../config/decxin_px4_flight.yaml) | 当前服务的视觉观测发布配置 |
| [decxin_px4.yaml](../config/decxin_px4.yaml) | 只接收飞控状态的诊断配置 |
| [px4-vio.service](systemd/px4-vio.service) | 当前定位服务环境与启动命令 |
| [pilot_navigation_nano.json](pilot-navigation/pilot_navigation_nano.json) | 遥控器授权任务配置 |

## 环境与安装路径

设备使用 Ubuntu 22.04 / aarch64、JetPack 6.2、ROS 2 Humble 和 PX4 1.17。
Agent 与消息工作区位于 `~/.cache/orbslam3-px4/`，其他设备依赖位于 `~/orb_dependencies/`。
构建步骤见 [构建说明](../docs/BUILDING.md)，各目录用途见 [硬件说明](docs/hardware.md)。

以下仓库路径均相对本目录：

| 仓库路径 | 设备路径 | 权限 |
|---|---|---|
| `bin/vio_daemon.sh` | `~/vio_daemon.sh` | 0755 |
| `bin/vio_inrec.py` | `~/vio_inrec.py` | 0755 |
| `bin/vio_restart_gate.py` | `~/vio_restart_gate.py` | 0755 |
| `bin/maps.sh` | `~/maps.sh` | 0755 |
| `bin/export_vio_image_ring.py` | `~/export_vio_image_ring.py` | 0755 |
| `bin/vio_image_retention.py` | `~/vio_image_retention.py` | 0755 |
| `bin/test_vio_ned_dds.py`、`bin/test_vio_image_tools.py` | `~/` 下同名文件 | 0755 |
| `sbin/vio-clock-prepare` | `/usr/local/sbin/vio-clock-prepare` | 0755，root |
| `sbin/vio-camera-prepare` | `/usr/local/sbin/vio-camera-prepare` | 0755，root |
| `systemd/px4-vio.service` | `/etc/systemd/system/px4-vio.service` | 0644，root |
| `systemd/nano-vio-camera-recovery.sudoers` | `/etc/sudoers.d/nano-vio-camera-recovery` | 0440，root |
| `pilot-navigation/` | `~/pilot_navigation/releases/<版本>/`，由 `current` 链接选择 | 脚本 0755，其余 0644 |
| `pilot-navigation/pilot_navigation_nano.json` | `~/.config/pilot-navigation.json` | 0644 |
| `pilot-navigation/pilot-navigation.service` | `/etc/systemd/system/pilot-navigation.service` | 0644，root |

`vio-camera-prepare` 仅匹配旧 D435i 的 USB ID，不承担 DECXIN 的 USB 恢复。
设备依赖、构建产物及原始内录不随本目录分发。

## 启动顺序

1. systemd 执行校时及相机准备脚本，启动 `vio_daemon.sh`。
2. 守护进程等待相机和飞控串口，启动 DDS Agent 并检查握手。
3. 加载 ROS 环境，启动连续状态内录，等待 `INREC_READY`。
4. 启用 IMU 时，检查新鲜的未解锁、着地证据，通过后启动 `stereo_uvc`。
5. `pilot-navigation.service` 独立等待导航状态与物理遥控器档位。

故障退出后，仍解锁时不会重启 VIO 建立新参考系。
正常停止依次结束 SLAM、排空内录、退出 Agent。看门狗阈值和恢复逻辑见 [守护说明](docs/daemon.md)。

## 日常查看

```bash
systemctl status px4-vio pilot-navigation
systemctl is-enabled px4-vio pilot-navigation
tail -f ~/vio_daemon.log
journalctl -u pilot-navigation -f
```

飞手依次使用 Altitude、Position、Offboard 三个物理档位。
完成 VIBA2、实际 EV 融合和 `OFFBOARD READY` 后才可执行五米任务；到点持续 HOLD，拨回人工档撤销任务。
完整操作与待验收项目见 [五米任务流程](../docs/DECXIN_FIVE_METRE_FLIGHT.md)。

## 运行产物

| 设备路径 | 内容 |
|---|---|
| `~/vio_daemon.log`、`~/vio_service.log` | 守护与定位服务日志 |
| `~/vio_agent.log` | DDS Agent 日志 |
| `~/ORB_SLAM3/recordings/test_runs/<run>/` | 运行日志、配置快照、导航记录与桥接遥测 |
| `~/inrec/` | 连续状态 CSV、分段与完整性摘要 |
| `~/vio_image_events/` | 按需冻结的图像事件 |
| `~/maps/` | Atlas 存档 |
| `~/.local/state/pilot-navigation/` | 任务控制 CSV 与实例锁 |

图像内录默认关闭，状态 CSV 不能代替视觉回放数据。完整性判断和导出方法见 [录制说明](docs/recording-and-maps.md)。

## 历史记录

[verification/](verification/) 按日期保存部署与验证记录，测试数量和结果只适用于对应版本。
旧 D435i 操作与飞控参数见 [历史资料](../docs/README.md#历史资料)。
`px4/` 中的 2026-11-05 参数文件为历史建议，不是当前 DECXIN 入口的参数清单。
