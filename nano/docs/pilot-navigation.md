# 三档遥控器授权的任务服务（2026-11-09）

DECXIN 当前服务已接入校准后的 PX4 原始 IMU；下面的历史部署记录不替代本轮验证。当前使用方式与限制见 [首次空中初始化与五米任务](../../docs/DECXIN_FIVE_METRE_FLIGHT.md)。


`pilot-navigation.service` 是独立于定位守护服务的第二个开机自启服务。它等待遥控器档位和有效
VIO，条件满足后才发 Offboard 轨迹；不解锁、不主动起飞、不切飞控模式、不强制降落。
`px4-vio.service` 的单元、守护脚本、VIO 配置、相机配置和校时脚本本轮均未改动，也未重启原定位服务、
未更新 SLAM 源码或二进制、未改飞控参数与固件。

用户确认的操作顺序是遥控器手动起飞：上档 Altitude、中档 Position、下档 Offboard。
Offboard 航线沿实际接管时的机头方向前进 5 m，速度 0.25 m/s，保持接管时高度，到位后 HOLD
等飞手接回。

## 版本与落地位置

| 仓库路径（本分支 `nano/` 下） | 设备路径 |
|---|---|
| `pilot-navigation/` 下 10 个文件 | `~/pilot_navigation/releases/20261109-5ae37233bf98/`；`~/pilot_navigation/current` 指向该版本 |
| `pilot-navigation/pilot_navigation_nano.json` | `~/.config/pilot-navigation.json` |
| `pilot-navigation/pilot-navigation.service` | `/etc/systemd/system/pilot-navigation.service` |
| — | `~/.local/state/pilot-navigation/` 放控制 CSV 与实例锁，运行期产物不入库 |

`pilot-navigation/` 除运行配置模板外与设备 release 目录逐字节一致，单元文件随版本放在同一目录，
不再复制到 `systemd/`，避免两处副本不同步。2026-11-09 晚间设备运行配置改为 5 m，本分支的
`pilot_navigation_nano.json` 同步更新，设备 release 目录内仍是 10 m 原版，变更记录见
`verification/20261109-5m/`。`systemd/` 仍只放定位服务的单元与 sudoers。

版本目录 10 个文件 SHA256 见 `verification/20261109/deployment_manifest.json`，清单记录的是
11-09 部署时的 10 m 原版；原有 5 份基线文件部署前后哈希一致，见同目录 `baseline_manifest.json`。

## 开机后的流程

1. 定位守护服务自动运行，等待相机与飞控 DDS 会话；SLAM 正常后持续采集，但不因「进程存在」
   就声称定位有效。
2. 遥控器置上档 Altitude，保持至少 1.2 s，以新鲜物理 RC 消息、开关槽位、飞控实际及意图模式
   识别上档。飞手自行解锁、控制起飞。
3. 当前配置启用首次空中建系。Altitude 下给出受控的平移/加减速激励，等待 IMU 初始化、BA2、
   时间与质量门控通过；仅悬停或固定等待时长不能保证完成。已有 VIO 故障后的空中重建不在此授权内。
4. 确认 VIO 与 PX4 融合稳定后切中档 Position，保持至少 1.2 s，确认位置控制。程序识别与上档
   不同的第二槽位。VIO 健康、位置、RC 及其余检查均通过，再连续预发 1.2 s
   `OffboardControlMode` 心跳，显示 `OFFBOARD READY`。
5. 看到 READY 后拨下档 Offboard。第三物理槽位必须不同于刚识别的 Altitude/Position，且飞控实际
   与用户意图都为 Offboard，才捕获当前目标锚点并开始航线。提前拨动要求切回中档、重新等 READY，
   不会稍后自动续入。
6. 任意时刻切上档或中档，撤销本次任务、清空目标、停止心跳与轨迹；不发 Land/Arm/Mode 指令。
   到位只报告完成并保持位置，仍需飞手切回。

识别两个人工档位避免猜测 PWM 正反向或把仿真的第 2 槽位套到实机。映射只保存在本进程内；
开机或手动重启后重新观察两个档位。档位映射冲突或已识别位置改变会阻止授权。开机等待无 600 s
超时，不因飞手准备时间长而退出。任务撤销或进程异常后不自动重新启动：单元为 `Restart=no`，
飞手接回并落地后再显式开始下一次任务。

配置 `pilot_navigation_nano.json` 取值：`rc_selector=three_position`、`mission=line`、
`distance_m=5.0`、`side_m=1.0`、`speed_m_s=0.25`、`max_radius_m=6.0`、
`max_alt_error_m=0.5`、`max_tracking_error_m=0.8`。启动脚本会校验参数边界，
速度上限 1 m/s，距离上限 30 m，取值条件为 `0 < value <= 30`，5 m 在范围内。
围栏半径保持比航线距离大 1 m，避免超限判定仍在 11 m 才触发。

## 查看状态

```bash
systemctl status px4-vio.service pilot-navigation.service
journalctl -u pilot-navigation.service -f

source /opt/ros/humble/setup.bash
source ~/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
ros2 topic echo /navigation/task_status
ros2 topic echo /vio/navigation_health
```

`phase=WAIT_PILOT` 表示正在等；`reason=observe_altitude_detent/observe_position_detent` 是三档
识别阶段；`rc_manual_slots` 显示已识别槽位。`offboard_ready=true`、`control_active=true` 分别表示
可以拨 Offboard、已接管；`task_completed=true` 表示到位 HOLD，不代表已交还飞手。缺少输入或定位
不合格时始终不发轨迹。

停止自动任务服务（定位服务继续运行）：

```bash
sudo systemctl disable --now pilot-navigation.service
```

恢复启动或开始下一次任务前先确认落地、人工模式与测试场地，再操作服务；不要在失去飞手接管
能力时靠远程 SSH 重启控制程序。

## 验证记录

证据原文在 `verification/20261109/`；改为 5 m 的记录在 `verification/20261109-5m/`。

| 检查 | 结果 / 限制 |
|---|---|
| Nano Ubuntu 22.04 / Humble / ARM64 逻辑与隔离 ROS 测试 | 44/44 通过；覆盖三档 PWM 正反、到位、提前切档、过期与重复数据、重定位撤销；测试只在隔离 domain 44、`/pilot_test` 中运行，未装仿真健康桩 |
| Nano 现有 VIO 门控 C++ 回归 | `ctest -R px4_vio_regression` 1/1 通过；软件用例覆盖首次空中 Altitude 接入，不是实飞 |
| systemd 单元静态检查 | 退出 0；系统已有 NVIDIA 单元的 syslog 废弃警告与新增服务无关 |
| 服务与进程 | 两个服务 enabled + active；定位服务原 MainPID 未重启；任务单元 `Restart=no` |
| 实际控制域只读观测 5 s | 25 条 WAIT 状态；0 心跳、0 轨迹、0 `VehicleCommand`；未尝试起飞 |
| 重启或断电自启动 | enabled 链接已核对，本轮未做重启或断电试验 |

## 未验证与已知限制

- 检查时 USB 没有 D435i，SLAM 未运行，DDS Agent 未建立会话，真实 VIO 健康与物理开关话题均无
  发布者。服务 active 只证明等待进程活着，不能据此起飞测试定位。
- 设备仍为 RealSense 专用采集入口，而配置几何值已改为 1360×960、fx/fy 667、30 Hz。实际 USB
  相机适配与实测标定需要独立确认；本轮未改参数，也未假设这套参数已可飞。
- 实机固件必须导出 `/fmu/out/manual_control_switches`；连接恢复后需核实实际话题，不能以 Python
  消息类存在当作飞控已发布。本轮未刷固件，配置建议见 `docs/rc_switch_dds.patch`。
- 设备 UTC 仍停留在 11 月 6 日且 NTP 未同步。本轮未对运行服务调系统时间；准确 UTC 仍需 RTC/NTP
  或停定位后同步，防止运行中跳时触发故障。
- 首次空中初始化可用不等于重定位后可空中无感续飞；参考变化或姿态跳变仍撤销并按现有定位门控处理。
- 实机 VIO 初始化、航线到位精度、接管时延、断电自启动均未验证；本轮无相机及有效 DDS 输入，
  11-09 的回归在 10 m 配置下完成，改为 5 m 后未重跑实机。
