# DECXIN / PX4 接入

当前使用 DECXIN 图像与 PX4 `SensorCombined` 原始 IMU，定位入口为 `stereo_uvc`。
服务启动和五米任务流程见 [五米任务说明](../../docs/DECXIN_FIVE_METRE_FLIGHT.md)。

## 配置入口

| 文件 | 用途 |
|---|---|
| [decxin_px4_imu.yaml](../../config/decxin_px4_imu.yaml) | 原始 IMU 接收、旋转外参与图像时间偏移 |
| [decxin_px4_flight.yaml](../../config/decxin_px4_flight.yaml) | 当前服务的外部视觉发布门控 |
| [decxin_px4.yaml](../../config/decxin_px4.yaml) | 只接收飞控状态的诊断配置 |
| [px4-vio.service](../systemd/px4-vio.service) | 常驻服务的实际环境变量 |

## 坐标与杆臂

`IMU.T_b_c1` 将校正后的左目光学坐标变换到 PX4 body FRD，已包含校正旋转和相机杆臂。
尺量平移为前 150 mm、右 20.5 mm、下 49.5 mm，安装旋转的验证见 [原标定记录](../../docs/DECXIN_EXTRINSICS_20261110.md)，当前时间偏移见 [YUYV 验证](../../docs/DECXIN_YUYV_20261113.md)。

当前桥接输出在 PX4 IMU 中心，因此该入口的 `EKF2_EV_POS_X/Y/Z` 应为 0。
不能再套用旧 D435i 的相机到内部 IMU 关系或杆臂补偿。
仓库记录尚未读取或修改当前实际飞控参数，需要单独核对。

## 时间与就绪条件

当前时间偏移约为 -52.280 ms，适用于 YUYV 2720×960 @ 30 fps、单 V4L2 缓冲及读出后软件时间戳；原 MJPG 的 -44.534 ms 只用于成套回退。
它是图像与原始 IMU 的时间对齐量，不能直接抄作 `EKF2_EV_DELAY`。

视觉观测发布需通过跟踪、VIBA2、时序、运动一致性和参考系检查。
任务就绪还需实际 EV 融合、有效位置与速度，以及物理遥控器档位。
首次空中参考、交接窗口、故障撤销和飞手操作均以 [五米任务说明](../../docs/DECXIN_FIVE_METRE_FLIGHT.md) 为准。

## 查看状态

```bash
source /opt/ros/humble/setup.bash
source ~/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
ros2 topic echo /vio/navigation_health
ros2 topic echo /navigation/task_status
```

结合导航健康、桥接遥测与实际融合标志判断状态，不能仅凭进程存在或图像有效判定可接管。
记录范围与完整性检查见 [录制说明](recording-and-maps.md)。

## 旧参数记录

[D435i 飞控记录](archive/d435i-flight-controller-20261105.md) 保存了早期的融合、120 ms 时延及杆臂说明。
`nano/px4/` 下的 2026-11-05 参数文件也属于该阶段，不能直接作为 DECXIN 当前配置导入。
