# DECXIN 首次空中初始化与五米任务

## 已部署的流程

px4-vio.service 启动校准后的 IMU_STEREO 入口，使用 config/decxin_px4_imu.yaml
及 config/decxin_px4_flight.yaml。GPU 校正和 FAST 保持开启，处理尺寸仍为每目
960×720、1000 配置特征。惯性入口强制单 V4L2 缓冲并关闭新取流流水线，
保留 -44.534 ms 时间标定对应的采集方式。

新 VIO 进程只允许在记录器提供新鲜、源时间也新鲜的“未解锁且着地”证据时启动。
程序可以在飞手随后起飞时继续惯性初始化，但故障退出后不会在仍解锁时重启建系。
连续状态内录保留原始 IMU、导航健康、行为树任务状态及事件。

首次空中外部视觉参考仅允许在 MANUAL/ALTCTL/STAB 中建立，必须满足
有效跟踪、惯性初始化、VIBA2、有限位姿和速度、运动一致性、时间同步、
新鲜且与图像时刻匹配的 PX4 数值局部坐标等检查。
不要求初次接入前 XY 已有有效位置，避免无视觉融合时形成循环依赖；
这不表示初始 XY 已经可靠。数值坐标和高度连续保持，不人工归零伪造位置。

初次采用 EV 后，原有 3 秒交接窗口仍允许 Altitude 中的首次 EKF 接入重置，
但在窗口结束前 navigation_ready 保持 false。
随后还要求 PX4 外部视觉位置融合、位置和速度有效、新鲜且参考系稳定。
首次参考建立后的地图/时钟/飞控坐标重置仍会锁存故障，不允许空中反复重建。
本配置把 airborne observation gap 上限收紧到 250 ms，没有放宽 IMU 的 30 ms 连续性要求。

融合状态 EstimatorStatusFlags 实测约 1 Hz；旧 0.5 秒新鲜度会在正常消息之间反复取消就绪。本次修正为 1.2 秒，并测试 1 Hz 正常保持、超过上限立即失效及 EV 标志撤回立即失效。位置、IMU、观测的时效要求未放宽。

## 飞手操作与可见状态

1. 上电后先保持未解锁、已着地，确认定位与任务服务运行。遥控器上档 Altitude
   至少保持 1.2 秒，让程序识别物理档位。飞手自行完成飞控前检与人工起飞。
   程序不发送解锁、起飞或模式切换指令。
2. 起飞后由飞手在 Altitude 中控制飞机；等待必要运动激励完成惯性初始化与 VIBA2。
   只悬停或等固定时长不能保证初始化完成。未就绪时不发任务轨迹。
3. VIO、EV 融合和局部位置持续正常后，飞手切中档 Position，确认位置保持，
   至少保持 1.2 秒让程序识别第二个物理档位。
4. 查看 /navigation/task_status：flight_stage=READY_FOR_OFFBOARD、
   offboard_ready=true，或日志出现 OFFBOARD READY，再拨下档 Offboard。
   这时已经连续预发至少 1.2 秒 OffboardControlMode 心跳；预发不含轨迹。
   飞控实际及意图模式均为 Offboard，物理第三档也匹配后，任务才捕获锚点。
5. 以接管时机头方向前进 5 m，速度 0.25 m/s，保持接管时高度和航向。
   到点需满足位置容差 0.15 m、速度不超过 0.2 m/s、连续停留 2 秒，
   并且平滑设定点已经到达目标，才进入 HOLD。
6. task_completed=true / phase=HOLD 后持续发布目标位置，直到飞手切回或门控撤销。
   不是到点自动降落。上/中档接回立即撤销任务；重新拨回不自动重跑。
   重启任务必须由飞手先接管、落地后显式操作。

提前拨 Offboard不会排队等待初始化后突然启动，必须切回人工档，
等 READY 后重新拨入。链路、健康、RC、位置源时间过期或参考改变会撤销任务，
停止心跳与轨迹，由 PX4 配置的 Offboard 丢失处置和飞手承担后续控制。
定位失效时不能保证“原地悬停”，不得将撤销任务理解为一定能定点保持。

## 状态查看

~~~bash
systemctl status px4-vio pilot-navigation
journalctl -u pilot-navigation -f

source /opt/ros/humble/setup.bash
source ~/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
ros2 topic echo /vio/navigation_health
ros2 topic echo /navigation/task_status
~~~

task_status 包含 flight_stage、navigation_ready、vio_gate_reason、
offboard_ready、task_completed、control_active、distance_m 和 speed_m_s。
WAIT_VIO 表示初始化/融合尚未就绪，VERIFY_RC_DETENTS 表示等待人工档位识别，
READY_FOR_OFFBOARD 才是可拨档状态。日志和完整 JSON 同时保存在内录中。

## 飞控参数与链路边界

当前只能通过 Nano 的 XRCE-DDS 链路读取飞行状态，未读取或修改实际飞控参数。
实飞前必须核对飞控 EV 融合、物理遥控器三档映射和 Offboard 丢失处置；
看到 navigation_ready 才说明运行中的位置融合门控实际通过。

ORB 的 IMU.T_b_c1 已包含前 150 mm、右 20.5 mm、下 49.5 mm 的相机杆臂，
桥接输出位姿位于 PX4 IMU 中心。因此本入口的 EKF2_EV_POS_X/Y/Z 应为 0，
不能再套用旧 D435i 相机/IMU 杆臂重复补偿。仓库旧 proposed-params-20261105.params
是历史建议，不是 DECXIN 当前入口参数清单，也没有被本次自动写入飞控。

内核记录过 CP2102 的 USB -32 错误、断连和重新枚举，造成真实 IMU 缺失；
设备 power/control 当前已为 on，不能将此故障简单归结为 autosuspend。
软件会拒绝跨缺口积分和导航，并阻止仍解锁时重启 VIO，不能修复物理断线。
短时正常接收不代表线缆问题已永久消失。

## 本轮验证

- 在 Nano 增量编译桥接门控和实际 C++ 回归，第一轮 2 个编译单元、第二轮 3 个编译单元，两轮各重链接 5 个目标。
- 28 项授权/启动用例及 18 项隔离 ROS 行为树用例通过；测试使用 domain 44，
  包括空中初始化等待、提前拨档拒绝、五米方向/高度/速度约束、
  到点持续 HOLD 和人工接回，未向真实飞控发送轨迹或命令。
- 首次空中接入、3 秒交接期间不报告导航就绪、交接后就绪、Position 中拒绝首次建系、
  空中参考变化锁存等 C++ 用例通过。
- sortie 回归 18 项、录制门控 10 项通过；真实 IMU 接收器的隔离 domain 45
  正常数据、裁剪、时间倒退、拒绝缺口和恢复检查通过。
- 重启门控 5 项、完整内录 3 项、启动器和 UVC 看门狗检查通过。
  内录验证覆盖陈旧源时间不能因新接收而续期、着地/解锁状态写入清单。
- 静置实机 45 秒检查 IMU 约 100 Hz，无非正或超过 30 ms 的采样间隔。
  程序显示 mode=vio，保持等待运动激励，未完成 VIBA2、未发布视觉观测。
- 行为树实际部署配置为前进 5 m、0.25 m/s、最大半径 6 m，
  高度偏差上限 0.5 m、跟踪误差上限 0.8 m。

这些是软件和台架证据，不是实飞放行或“程序不存在任何问题”的保证。
尚未完成当前配置在空中的初始化、实际 EV 融合、长期连续性和五米闭环飞行验收。
