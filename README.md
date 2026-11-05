# Orin Nano 侧部署

机载计算机上的部署物与现场记录：systemd 单元、守护脚本、D435i 恢复程序、录制与导出
工具、飞控参数建议。

VIO 源码、飞行与建图两套 profile 在 `master`，这里不复制。相机内参、外参、桥接阈值
都以 `master` 里的 yaml 为准，本分支只记设备侧怎么装、怎么起、出问题怎么判。

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

## 目录

```
bin/       守护脚本、内录、地图管理、图像导出与清理、两个自测脚本
sbin/      vio-camera-prepare，只重新枚举 D435i
systemd/   px4-vio.service 与单条 sudoers 授权
px4/       待写入飞控的参数，经 QGC 导入
docs/      硬件与软件栈、守护逻辑、录制与地图、飞控侧配置
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

## 自愈

`watch_stall` 每 15 秒看一次，两路证据都停才判卡死：

- `[VIO_PERF]` 计数 45 秒不增长 → 图像没进主循环。先冻结最后有效输入，再重启采集，
  并留下 `.vio_camera_recover`，下一轮做定向 USB 恢复。
- 日志行数与进程累计 CPU 时间同时不变 90 秒 → 判死，结束本次会话由 systemd 重启。

只数日志行会误判：等 IMU 初始化时日志本来就长时间不增长。

`vio-camera-prepare` 只处理 `8086:0b3a`，做法是 unbind/bind 让内核重新枚举，不重载
驱动；目标设备正被 VIO 使用时返回 `skipped_running_vio`，避免形成「复位 → SLAM 崩 →
再复位」的循环。细节见 `docs/daemon.md`。

## 录制、地图与飞控

- 环形图像、事件冻结、导出：`docs/recording-and-maps.md`
- 飞控侧 EKF2、时延、杆臂与判读：`docs/flight-controller.md`
- 待写入的参数：`px4/proposed-params-20261105.params`（尚未写入飞控）
- 开机即录 ULog：`px4/boot-ulog-20261105.params`（`SDLOG_MODE=2`）

## 已知限制

- 系统时间：板载 RTC 读出 1970，`fake-hwclock` 记下的时间开机被内核 `hctosys` 覆盖，
  录出来的文件名是 `inrec_19700101_*`。时间同步相关的检查项因此必然报未就绪。
- 25 W 功耗墙下能否长时间跑满 1728 MHz 未验证，空载观测到的 1190–1344 MHz 属正常 DVFS。
- 环形缓冲只在磁盘：最近 20 秒全帧率，另每约 1 秒留一组作 120 秒背景。背景里的 IMU
  批次不连续，只能看场景，不能连续复跑惯性。
- 20 秒片段不是完整 SLAM 状态快照，缺少初始化激励时无法独立复现跟踪状态。
- 5 cm 基线在弱纹理、强反光、远距离和机体振动下可观测性都差，加特征点解决不了。

## 分支

| 分支 | 内容 |
|---|---|
| `master` | ORB-SLAM3 源码与两套 profile |
| `nano` | 本分支，设备侧部署物与现场文档，与 master 无共同历史 |
| `data/first-crash-20261103` | 首次炸机那一次的遥测、内录与分析 |

设备源码更新走 master，部署物改动只落在本分支；两边路径不同，互不覆盖。
