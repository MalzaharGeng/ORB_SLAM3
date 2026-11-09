# 试验机 Nano 侧部署

试验机 Nano（Orin Nano Super）上的部署物与现场记录：systemd 单元、守护脚本、D435i 恢复程序、录制与导出
工具、飞控参数建议，以及 2026-11-09 起独立运行的三档遥控器授权任务服务。

2026-11-06 起本分支与 `master` 同源，仓库根目录是试验机 `/home/nano/ORB_SLAM3` 的源码快照；
本目录 `nano/` 只保留设备侧怎么装、怎么起、出问题怎么判。相机内参、外参与桥接阈值仍以根目录
`Examples/Stereo-Inertial/`、`config/` 下的 yaml 为准。

## 环境

| 项 | 值 |
|---|---|
| 板卡 | Jetson Orin Nano Engineering Reference Developer Kit Super，6× Cortex-A78AE |
| 主频 | 最高 1728 MHz，`nvpmodel` 25 W 档 |
| 内存 / 存储 | 7.4 GiB / 233 GB |
| 系统 | Ubuntu 22.04.5 LTS aarch64，内核 5.15.148-tegra，JetPack 6.2（R36.4.7） |
| 相机 | D435i（`8086:0b3a`），双目 IR 640×480 @ 15 fps |
| 链路 | CP2102 USB-UART → `/dev/ttyUSB0`，921600，uXRCE-DDS |
| 飞控 | PX4 1.17 |
| 中间件 | ROS 2 Humble，MicroXRCEAgent 2.4.3，librealsense 2.56.5（V4L2 后端） |

开机不依赖网络，也不依赖图形桌面。图形栈开不开都不影响服务。

本地开发用的 WSL 是另一套环境：Ubuntu 26.04 / x86_64 / ROS 2 lyrical / `px4_msgs` 1.17.0。
`src/Px4Bridge.cc` 的消息版本断言在本地不成立，本地只能关闭 PX4 桥接编译核心逻辑与离线回归，
CTest 结果不代表设备端构建。源码与配置文本可以双向比对；**本地 WSL 的编译产物、依赖配置、
系统服务配置一律不向设备同步**。任何 `WSL → 设备` 写入或 `master → 设备` 部署，
以及把 master 新提交合并进 `nano` 分支，均需逐次明确授权。两端对照见根目录 README 的
「运行环境与同步边界」。

## 相关位置

源码、运行配置和数据都不在本分支，按下表找：

| 内容 | 位置 |
|---|---|
| SLAM 源码 | 本分支仓库根目录（与 `master` 同源）；设备上 `/home/nano/ORB_SLAM3` |
| 飞行配置 | [master: Examples/Stereo-Inertial/DECXIN_Camera.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/DECXIN_Camera.yaml) |
| 地面建图配置 | [master: Examples/Stereo-Inertial/RealSense_D435i_mapping.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/Examples/Stereo-Inertial/RealSense_D435i_mapping.yaml) |
| 桥接配置（飞行） | [master: config/px4_vio_local.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio_local.yaml) |
| 桥接配置（建图） | [master: config/px4_vio.yaml](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/config/px4_vio.yaml) |
| 启动器 | [master: run_recovery_test.sh](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/run_recovery_test.sh)；设备上 `/home/nano/ORB_SLAM3/run_recovery_test.sh` |
| 桥接实现 | [master: src/Px4Vio.cc](https://github.com/MalzaharGeng/ORB_SLAM3/blob/master/src/Px4Vio.cc) |
| 首次炸机的数据与分析 | [data/first-crash-20261103](https://github.com/MalzaharGeng/ORB_SLAM3/tree/data/first-crash-20261103) |
| 三档任务服务 | 本分支 [pilot-navigation/](pilot-navigation/)；设备上 `~/pilot_navigation/current`；流程与边界见 [docs/pilot-navigation.md](docs/pilot-navigation.md) |
| DDS Agent 与消息包 | 设备上 `/home/nano/.cache/orbslam3-px4/`，不进任何仓库 |
| 解包依赖 | 设备上 `/home/nano/orb_dependencies/`，不进任何仓库 |
| 本分支快照 | [nano-deploy-20261105](https://github.com/MalzaharGeng/ORB_SLAM3/releases/tag/nano-deploy-20261105) |

## 目录

```
bin/              守护脚本、内录、地图管理、图像导出与清理、两个自测脚本
sbin/             vio-camera-prepare（重新枚举 D435i）、vio-clock-prepare（启动校时）
systemd/          px4-vio.service 与单条 sudoers 授权
pilot-navigation/ 2026-11-09 三档任务服务版本目录：脚本、测试、配置与单元
px4/              待写入飞控的参数，经 QGC 导入
docs/             硬件与软件栈、守护逻辑、录制与地图、飞控侧配置、三档任务流程
verification/     部署与验证记录，按日期分目录
```

## 部署

| 仓库路径 | 设备路径 | 权限 |
|---|---|---|
| `bin/vio_daemon.sh` | `/home/nano/vio_daemon.sh` | 0755 |
| `bin/vio_inrec.py` | `/home/nano/vio_inrec.py` | 0755 |
| `bin/maps.sh` | `/home/nano/maps.sh` | 0755 |
| `bin/export_vio_image_ring.py` | `/home/nano/export_vio_image_ring.py` | 0755 |
| `bin/vio_image_retention.py` | `/home/nano/vio_image_retention.py` | 0755 |
| `bin/test_vio_ned_dds.py` | `/home/nano/test_vio_ned_dds.py` | 0755 |
| `bin/test_vio_image_tools.py` | `/home/nano/test_vio_image_tools.py` | 0755 |
| `sbin/vio-camera-prepare` | `/usr/local/sbin/vio-camera-prepare` | 0755，root |
| `systemd/px4-vio.service` | `/etc/systemd/system/px4-vio.service` | 0644 |
| `systemd/nano-vio-camera-recovery.sudoers` | `/etc/sudoers.d/nano-vio-camera-recovery` | 0440，root |
| `pilot-navigation/*`（版本目录 10 文件） | `~/pilot_navigation/releases/20261109-5ae37233bf98/`，`current` 软链接指向 | 0644；`run_pilot_navigation.sh` 0755 |
| `pilot-navigation/pilot_navigation_nano.json` | `~/.config/pilot-navigation.json` | 0644，nano |
| `pilot-navigation/pilot-navigation.service` | `/etc/systemd/system/pilot-navigation.service` | 0644，root |

```bash
sudo install -m 0755 bin/*                   /home/nano/
sudo install -m 0755 sbin/vio-camera-prepare /usr/local/sbin/
sudo install -m 0644 systemd/px4-vio.service /etc/systemd/system/
sudo install -m 0440 -o root -g root systemd/nano-vio-camera-recovery.sudoers \
     /etc/sudoers.d/nano-vio-camera-recovery
sudo visudo -c
sudo systemctl daemon-reload
sudo systemctl enable --now px4-vio.service
```

上面的命令只装定位服务。三档任务服务按版本目录整包下发，单元与配置分别安装到上表路径，
`current` 指向该版本后再 enable；安装状态与验证输出见
[verification/20261109/](verification/20261109/)。不要在失去飞手接管能力时靠远程 SSH 重启控制程序。

`vio_daemon.sh` 要求 `~/ORB_SLAM3` 已构建（`build_nano`），且 `~/.cache/orbslam3-px4/`
下有 MicroXRCEAgent 和 ros2 消息工作区。这两样不在本分支，来源写在
`docs/hardware.md`。

sudoers 只放行 `vio-camera-prepare ""` 一条无参数命令，不给通用 sudo。

## 启动时序

```
systemd → fake-hwclock load
        → vio-camera-prepare            重新枚举 D435i，VIO 在跑时跳过
        → vio_daemon.sh
             ├─ 等相机节点（video ≥ 6、hidraw ≥ 1），再留 5 秒稳定窗口
             ├─ 起 MicroXRCEAgent，握手成功才继续，失败整只重启，最多 3 次
             ├─ 起 vio_inrec.py              同步内录到 ~/inrec/
             └─ 起 stereo_inertial_realsense_D435i --console，stdin 由 FIFO 顶住
```

节点齐了不等于控制通道就绪：刚枚举完立刻开流会 `get_xu(EAGAIN)` 直接退出，所以有
`CAM_SETTLE`（默认 5 秒）。PX4 上电初期串口没有数据，Agent 自己不会重试，只能由
守护脚本整只重启。

服务停止走 `trap`：先停采集、收尾内录、再退 Agent。只删 pidfile 的写法会让
`systemctl stop` 等到超时。

## 三档任务服务

`pilot-navigation.service` 与定位服务并列开机自启，只等待遥控器档位与有效 VIO，不解锁、
不主动起飞、不切飞控模式、不强制降落。上档 Altitude 手动起飞并完成 VIO 初始化，中档 Position
确认定位并等 `OFFBOARD READY`，下档 Offboard 执行沿实际接管时方向 5 m、0.25 m/s 的航线；
拨回上档或中档立即撤销任务，单元为 `Restart=no`，不会自动续飞。两个人工档位每次开机重新观察，
流程、状态话题、参数与未验证边界见 [docs/pilot-navigation.md](docs/pilot-navigation.md)；
2026-11-09 晚间改为 5 m 的记录见 [verification/20261109-5m/](verification/20261109-5m/)。

## 运行期产物

| 路径 | 内容 |
|---|---|
| `~/vio_daemon.log` | 守护脚本日志 |
| `~/vio_service.log` | 服务 stdout/stderr |
| `~/vio_agent.log` | DDS Agent 输出 |
| `~/ORB_SLAM3/recordings/test_runs/<run>/` | `run.log`、`px4_telemetry.csv`、`camera_calibration.json`、`image_ring/` |
| `~/inrec/inrec_*.csv` | 同步内录，与图像缓冲独立 |
| `~/vio_image_events/` | 冻结的事件片段 |
| `~/maps/` | Atlas 存档，`active` 指向下次启动加载的那张 |
| `~/.local/state/pilot-navigation/` | 任务服务控制 CSV 与实例锁，不入库 |

## 自愈

`watch_stall` 每 15 秒看一次，两路证据都停才判卡死：

- `[VIO_PERF]` 计数 45 秒不增长 → 图像没进主循环。先冻结最后有效输入，再重启采集，
  并留下 `.vio_camera_recover`，下一轮做定向 USB 恢复。
- 日志行数与进程累计 CPU 时间同时不变 90 秒 → 判死，结束本次会话由 systemd 重启。

只数日志行会误判：等 IMU 初始化时日志本来就长时间不增长。

`vio-camera-prepare` 只处理 `8086:0b3a`，做法是 unbind/bind 让内核重新枚举，不重载
驱动；目标设备正被 VIO 使用时返回 `skipped_running_vio`，避免形成「复位 → SLAM 崩 →
再复位」的循环。细节见 [docs/daemon.md](docs/daemon.md)。

## 录制、地图与飞控

- 环形图像、事件冻结、导出：[docs/recording-and-maps.md](docs/recording-and-maps.md)
- 飞控侧 EKF2、时延、杆臂与判读：[docs/flight-controller.md](docs/flight-controller.md)
- 待写入的参数：[px4/proposed-params-20261105.params](px4/proposed-params-20261105.params)（尚未写入飞控）
- 开机即录 ULog：[px4/boot-ulog-20261105.params](px4/boot-ulog-20261105.params)（`SDLOG_MODE=2`）

## 已知限制

- 系统时间：板载 RTC 读出 1970，`fake-hwclock` 记下的时间开机被内核 `hctosys` 覆盖，
  录出来的文件名是 `inrec_19700101_*`。时间同步相关的检查项因此必然报未就绪。
- 25 W 功耗墙下能否长时间跑满 1728 MHz 未验证，空载观测到的 1190–1344 MHz 属正常 DVFS。
- 环形缓冲只在磁盘：最近 20 秒全帧率，另每约 1 秒留一组作 120 秒背景。背景里的 IMU
  批次不连续，只能看场景，不能连续复跑惯性。
- 20 秒片段不是完整 SLAM 状态快照，缺少初始化激励时无法独立复现跟踪状态。
- 5 cm 基线在弱纹理、强反光、远距离和机体振动下可观测性都差，加特征点解决不了。
- 2026-11-09 的任务服务只完成 44 项逻辑与隔离 ROS 回归、1 项 VIO 门控 C++ 回归和 5 s 只读观测；
  当时无 D435i 与有效 DDS 输入，实机 VIO 初始化、十米精度、接管时延、断电自启动均未验证。

## 分支

| 分支 | 内容 |
|---|---|
| `master` | 默认分支，源码主线 |
| `nano` | 本分支，第二长期分支；与 `master` 同源，含同一份源码与 `nano/` 设备侧结构 |
| `nano-crash-20261103` | 从本分支分出，2026-11-03 首次炸机的遥测、内录与分析 |
| `nano-crash-20261105` | 从本分支分出，2026-11-05 两次飞行的 ULog、内录、分析与修复记录 |
| `data/first-crash-20261103`、`data/second-crash-20261105` | 早期独立历史分支，与对应 tag 一起保留，不再更新 |

设备源码改动先落 `master`；同步到本分支需逐次确认，设备侧部署另行授权。部署物改动落在本分支 `nano/` 下。
本分支基底早于当前 `master`，目前未做合并；`nano/sbin/vio-clock-prepare` 与 master `system/vio-clock-prepare` 内容一致。
`nano-deploy-20261105` 之前的独立历史由同名 tag 与 `archive/nano-deploy-20261106` 保留。
2026-11-09 的三档任务服务部署物由设备版本目录直接同步到本分支 `nano/`，未合入 `master`，
也未从本地 WSL 或 master 向设备写入。
