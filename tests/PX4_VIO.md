# PX4 1.17 VIO 接入

ROS 2 Humble → Agent 2.4.3 → 飞控 USB CDC → EKF2。只提供视觉里程计和状态回读，不发送飞行控制命令。

## 编译与启动

Avi 原验证环境将依赖安装到 `~/.cache/orbslam3-px4`；本分支本轮没有安装这些依赖。部署说明使用 Ubuntu 22.04 / ROS 2 Humble，并准备 CMake、编译器、colcon、Asio、TinyXML2、spdlog 开发包：

```bash
python3 scripts/setup_px4_dependencies.py --messages --agent
source /opt/ros/humble/setup.bash
source ~/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
cmake -S . -B build -DORB_SLAM3_BUILD_TESTS=ON -DORB_SLAM3_WITH_PX4=ON
cmake --build build --target stereo_inertial_realsense_D435i d435i_recovery_replay -j2
```

`px4_msgs` 固定为 `release/1.17` 的 `86d8239e962f6939e05c3737784f60c02fa884db`；Agent 固定 v2.4.3 的 `73622810d984349b80bbac0ef55fc0b694d62222`。完整依赖版本见安装目录的 `versions.json`。自编译飞控若改过消息定义，需要使用该固件的定义重新编译消息包；不要混用 `main`。Agent 使用独立 FastDDS/FastCDR，**不要把 agent_install/lib 加入 VIO 的 LD_LIBRARY_PATH**。

将 `config/px4_vio.yaml` 复制成自己的安装配置，只需填写 3×3 旋转矩阵 `R_BI`，然后设置 `extrinsics_valid: 1`。默认值 0 只回读状态，不发定位。旧配置 `T_BI` 需取左上角 3×3 旋转改写为 `R_BI`，平移只在飞控端设置。

```bash
# 终端 1；替换为实际设备标识，账户须具有串口读写权限
python3 scripts/run_px4_agent.py --device /dev/serial/by-id/<飞控标识>

# 终端 2；先 source 上述两个 ROS 环境
./run_recovery_test.sh --headless --px4-config config/px4_vio.yaml
```

桥接编译开关默认 OFF。关闭后无需 ROS 即可编译原程序；传入 `--px4-config` 会明确提示重新启用编译。通信线程独立运行，只缓存最新观测。Agent 启动器检查设备重枚举并重启 Agent；不自动改权限、系统服务或飞控参数。

## 飞控 USB 配置

通过当前可用的控制台设置 `SYS_USB_AUTO=0`、`UXRCE_DDS_CFG=0` 并保存；将 `config/px4_usb/extras.txt` 的块追加到 SD 卡 `/fs/microsd/etc/extras.txt`，然后重启。已有 extras 内容须保留。

扩展启动 USB 设备节点和 `uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600`，使用 domain 0、空命名空间、`UXRCE_DDS_SYNCT=1`。USB CDC 的 921600 是接口配置值，不代表 USB 物理链路吞吐率。不要同时启动第二个 DDS 客户端或让 QGC 占用该 USB CDC 口。

EKF 基线：`EKF2_EV_CTRL=7`、`EKF2_EV_NOISE_MD=0`、`EKF2_EV_QMIN=0`、`EKF2_EV_DELAY=0`；`EKF2_EV_POS_X/Y/Z` 使用下述安装平移。保留现有航向来源、高度参考及其他传感器融合配置；延迟与噪声初值需据实测调整。

恢复原生 USB MAVLink：删除 extras 中仅 DDS 对应的块，通过其他控制台执行 `config/px4_usb/restore_usb.txt`，重启。若没有其他控制台，可先用 SD 卡临时启动命令恢复 `SYS_USB_AUTO=2`，随后移除临时命令。USB 冷启动/插拔必须在目标板实际验收。

## 坐标、时间和状态约定

`R_AB` 表示 B 到 A 的旋转；`I` 是 D435i IMU，`B` 是飞行器 FRD，原点为机体重心。桥接只配置 `R_BI`，满足 `v_B = R_BI * v_I`。平移是 D435i IMU 中心相对机体重心的位置，仅填写到飞控 `EKF2_EV_POS_X/Y/Z`，单位米、轴向为前/右/下。这里不是相机光心，也不是简单的两块 IMU 间距离。它与相机 YAML 中的 `IMU.T_b_c1` 是不同外参；示例单位矩阵不是安装标定。

桥接取同一地图更新锁内的 `NavigationState`，增加 `velocityWorld`、`gravityWorld`、`inertialInitialized`、`inertialBA2`、`velocityValid`、`referenceEpoch`、`frameId` 和 `trackingState`。原有导航 API 坐标语义不变。速度来自 `Frame::GetVelocity()`，重力由惯性地图中的 `(0,0,-g)` 转入导航系；不使用位姿差分。

建立通信世界系 F 时，以重力方向为 Z、机头在水平面的投影为 X，Y=Z×X，原点为首次允许发布时的 IMU 中心：

```text
R_NB = R_NI * transpose(R_BI)
p_FI = R_FN * (p_NI - p_NI0)
v_FI = R_FN * v_NI
R_FB = R_FN * R_NB
```

发送位置和速度属于 **IMU 测量点**，姿态属于 **机体 FRD**。初始发送位置为零，之后保留 IMU 中心的真实运动；PX4 使用 `EKF2_EV_POS_X/Y/Z` 统一补偿位置和转动引起的速度杆臂。世界系是任意航向的重力对齐 FRD，不声称为真北 NED；PX4 自身位置原点不保证与这里相同。

通信启用时，RealSense 图像和陀螺仪/加速度计都要求 `GLOBAL_TIME`。四个流分别检查；时间域切换、倒退、任一流停顿超过 200 ms 或主机时钟跳变会清空缓存并重置惯性地图。SLAM 内部使用统一相对时间以避免大 epoch 数值；发送时恢复图像的主机采样时间。`timestamp_sample` 为采样时刻，`timestamp` 为发送时刻，单位微秒。PX4 XRCE 自动转换两者，不手动减 `timesync_status` 偏移。没有 Global Time 支持时拒绝启用本桥接。

发布要求：外参已确认、有效速度、IMU/VIBA2 完成、连续有效跟踪 1 s、飞控状态新鲜且有 DDS 订阅端。首次建立参考系必须未解锁。状态超时 1 s、DDS 同步超时 2 s、同步 RTT 大于 20 ms 或观测/估计偏移差大于 5 ms 时停发；每帧只发一次，超过 200 ms 的结果丢弃，不补发旧帧。

跟踪失效立即停发，恢复后重新满足 1 s 连续有效条件。同一地图的短暂通信断线保持参考系；地图重置、重定位、回环/GBA 校正、地图合并、首次地图对齐和时钟异常使参考失效，未解锁且稳定后重新建系，并增加 `reset_counter`。普通局部 BA 不触发该计数。首版不支持飞行中跨地图无缝恢复。

| 话题 | 用途 |
|---|---|
| `/fmu/in/vehicle_visual_odometry` | FRD 位置、机体到世界系四元数 wxyz、FRD 速度、方差、重置计数 |
| `/fmu/out/vehicle_status_v1` | 解锁、模式 |
| `/fmu/out/vehicle_local_position_v1` | EKF 状态及有效性 |
| `/fmu/out/estimator_status_flags` | EV 融合与拒绝标志 |
| `/fmu/out/timesync_status` | 仅使用 DDS 来源的时钟同步 |

话题版本后缀从消息常量生成。发布为 BestEffort/Volatile/KeepLast(1)，订阅为 SensorData QoS。未提供角速度填 NaN；quality=0，门控不依赖虚构质量分。方差是可配置标准差的平方，当前不是 ORB-SLAM3 后验协方差。控制台与 CSV 记录发送量、实测频率、时龄、丢弃原因、EV 标志和本地位置有效性；EV 标志为融合状态，不等同于绝对精度。

## 验证与复现

```bash
cmake --build build --target px4_vio_regression px4_bridge_fixture -j2
ctest --test-dir build --output-on-failure

# 回放订阅端：独立 domain 42 /vio_test
python3 tests/px4_bridge_ros_regression.py --serve /tmp/vio_replay_peer.json
# 另一终端用测试安装配置运行；回放程序强制隔离 domain/namespace
./run_recovery_test.sh --replay --headless --px4-config /path/to/test-extrinsics.yaml
```

回放的采样时间重映射为每帧注入主机的时刻，用于验证状态与数据路径，不代表原始采集延迟。默认回放 stride=2，因此约 15 Hz；实机使用每个实际处理的 30 Hz 图像结果，达不到时不重发凑频率。

PX4 SIH 测试使用 **nolockstep** 构建，避免虚拟仿真时钟随调度停顿漂移，而桥接与 Agent 使用真实主机时钟。构建辅助脚本固定上游源码，下载依赖仅在用户缓存中：

```bash
python3 -m venv --system-site-packages ~/.cache/orbslam3-px4/venv
source ~/.cache/orbslam3-px4/venv/bin/activate
pip install ninja kconfiglib 'empy>=3.3,<4' pyros-genmsg pyulog pymavlink jinja2 pyyaml jsonschema packaging
python scripts/setup_px4_sitl.py
python tests/px4_sitl_regression.py --fixture build/tests/px4_bridge_fixture \
  --px4-build ~/.cache/orbslam3-px4/PX4-Autopilot/build/px4_sitl_nolockstep \
  --output /tmp/vio-px4-sitl-test
```

SITL 使用 instance 17、domain 44、`/vio_sitl`、UDP 18888，不操作实机；它测试实际 PX4 EKF 对合成观测的融合。结果包括 PX4/Agent/桥接日志、接收记录和 ULog。源码 `snapshot_manifest.json` 记录真正上游提交；本地空 Git 元数据仅供官方构建脚本生成版本头，不是上游提交。

当前证据及限制见 `tests/PX4_VALIDATION.md`。接口依据：[PX4 1.17 uXRCE-DDS](https://docs.px4.io/v1.17/en/middleware/uxrce_dds.html)、[VehicleOdometry](https://docs.px4.io/v1.17/en/msg_docs/VehicleOdometry.html)。
