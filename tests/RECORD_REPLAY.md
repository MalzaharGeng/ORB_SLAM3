# 实机录制与按目录回放

在 `/home/lyx/ORB_SLAM3` 执行。`--record` 可与 GUI、无 GUI、SSH、PX4 桥接组合：

```bash
# 实机无 GUI，同时录制
./run_recovery_test.sh --headless --record

# 实机 GUI，同时录制
./run_recovery_test.sh --record

# 实机无 GUI，录制并启用已有 PX4 桥接配置
source /opt/ros/humble/setup.bash
source /home/lyx/.cache/orbslam3-px4/ros2_ws/install/local_setup.bash
./run_recovery_test.sh --headless --record --px4-config config/px4_vio.yaml
```

程序打印本次运行目录和可复制的回放命令。录制随程序运行，输入 `quit`、Ctrl+C 或 GUI Stop 正常结束后，会等待写盘完成并打印 `[Recording] COMPLETE frames=...`。

无需为了录制而输入 `first`。只运行定位、保存一张地图，或走完整个 `first → reset → second → load` 流程，都可以保存和回放。阶段推进条件保持原样；记录通过 GUI 或终端执行成功的阶段操作及显式地图重置，不记录 `status`、`pose` 等查询。

```text
recordings/test_runs/2026..._live_.../
  run.log
  launch.txt
  settings.yaml
  px4.yaml                    # 启用 PX4 时
  recording/
    format.txt
    settings.yaml             # 录制所用相机/IMU 配置
    stereo.csv                # 帧序号、时间戳、左右图路径、IMU 数量
    imu.csv                   # 按输入帧分组的同步 IMU 测量
    events.csv                # 阶段操作发生在哪一帧之前
    left/*.png
    right/*.png
    complete.txt              # 正常写完后的帧数；不存在表示未正常结束
    error.txt                 # 仅在录制写入失败时出现
```

指定运行目录或其中 `recording/` 均可；新的回放日志另建目录，不覆盖源录制：

```bash
./run_recovery_test.sh --replay /完整路径/某次运行目录 --headless
./run_recovery_test.sh --replay /完整路径/某次运行目录/recording
```

第一条无 GUI，第二条有 GUI。新格式固定逐帧回放，使用录制中的 YAML、图像时间戳、每帧的完整 IMU 批次及阶段记录。没有操作记录时只运行跟踪，不强制要求完成 VIBA、存图或读档。仍可用终端 `status`、`pose`、`quit` 查看或提前退出。

`--replay` 不带目录仍使用原来的 verified 录制和自动恢复测试，图像步长仍为 2。`--record --replay [目录]` 也可组合，把本次实际回放输入另存一份，用于验证录制链路；它不连接相机。一键回放会清除继承的 `ORB_REPLAY_UNPACED`，按传感器时间间隔播放。

## 记录内容和结果含义

录制的是 **实际送入 TrackStereo 的输入**：无损双目红外 PNG，以及实时程序已经同步到陀螺仪时间的加速度/角速度数据。包括 IMU 批次的边界和可能晚于图像时间的样本，回放不再次插值，也不重新按图像时间筛掉样本。

这不是 librealsense 全流 DB3 录包：没有保留实时程序未处理的相机帧，也不含彩色/深度流、飞控传感器和通信状态。该格式无需 ROS bag 导出步骤。需要 SDK 原始全流录包时仍使用原有 `d435i_capture_slam` 工具。

PNG 编码和磁盘写入使用独立线程，队列最多 60 帧。队列耗尽或写入失败会明确打印 `FAILED`、生成 `error.txt`，停录并让 SLAM 继续；正常退出返回非零表示录制失败，不静默丢帧。录制会增加 CPU、内存和磁盘负担，帧率应按目标设备实测。

未正常关闭的录制，没有 `complete.txt` 时可尝试回放已经完整写入的帧前缀，程序会警告未完成；若完整帧引用的图像/IMU 缺失或存在 `error.txt`，会报错而不是假装完整。正常结束、零帧的目录无法回放。

| 输出 | 含义 |
|---|---|
| `[Recording] COMPLETE` | 本次已接收输入写盘完成；不代表 SLAM 初始化或恢复成功 |
| `RECORDING_REPLAY_COMPLETE` | 已消费全部可读帧及记录的操作；打印有效位姿数量和最终阶段 |
| `RECOVERY_REPLAY_PASS` | 录制包含读档操作，回放也实际匹配成功 |
| `REPLAY_STOPPED_BY_USER` | 主动提前停止，不等于完整播放或恢复通过 |
| `REPLAY_FAILED` / `REPLAY_STAGE_INCOMPLETE` / `RECOVERY_NOT_ALIGNED` | 输入损坏，或阶段/最终几何匹配未完成 |

回放线程调度与原运行可能不同，因此存图、确认第二次初始化和读档以记录时刻作为最早请求时间，必要时打印等待并在真实条件满足后执行。**重置时间不延后**：如果到原记录的首次重置边界仍未初始化并存图，明确失败，不借第二段运动完成第一张地图；到输入结束仍有未完成操作也会失败。恢复后的长期精度、参数收敛仍需单独验证。

开启 PX4 桥接时，实时录制不改变发布条件；回放只有显式再加 `--px4-config FILE` 才启用桥接，并继续强制使用 domain 42、`/vio_test`。采样时间重映射为回放注入时间，不向实机 domain 0 重发历史定位。

## 直接调用与检查

一键脚本的 `--record` 不接受目录参数，自动保存在本次运行目录。直接二进制的 `--record DIR` 要求指定尚不存在的目录：

```bash
./Examples/Stereo-Inertial/stereo_inertial_realsense_D435i \
  Vocabulary/ORBvoc.txt Examples/Stereo-Inertial/RealSense_D435i.yaml \
  d435i_recovery --headless --console --record /新的绝对路径/recording

./build/tests/d435i_recovery_replay \
  Vocabulary/ORBvoc.txt /录制路径/recording/settings.yaml \
  /录制路径/recording /新的绝对路径/replay_output

./run_recovery_test.sh --record --headless --check
./run_recovery_test.sh --replay /录制路径 --headless --check
ctest --test-dir build --output-on-failure -R '^input_recording_regression$'
```

新格式不接受 `ORB_REPLAY_STRIDE` 大于 1；`ORB_REPLAY_VIEWER`、`ORB_REPLAY_CONSOLE`、`ORB_REPLAY_UNPACED` 可供直接调用使用，按是否存在判断，设为 0 仍是开启。通用录制回放的 `timing.csv` 为每帧读图/TrackStereo/周期耗时，`navigation.csv` 保留有效性、初始化状态及 IMU 位姿；VIBA 多组统计仍使用 `benchmark_recovery.py` 的历史输入路径。

2026-11-01 验证：7 项回归测试通过，历史 D435i 输入录制后按新目录完整恢复回放通过，无 first 的 60 帧录制在无 GUI/PX4 参数组合及本机 GUI 下通过。无新相机/飞控实测；详细日志和失败尝试保留在 [验证记录](../recordings/test_runs/input_recording_validation_20261101/README.md)。
