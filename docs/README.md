# 文档目录

本分支当前使用 DECXIN 双目相机与 PX4 原始 IMU，常驻处理为每目 960×720、nFeatures 1000。
日常操作从 [操作手册](../操作手册.md) 开始。

## 设备与运行

| 文档 | 内容 |
|---|---|
| [Nano 部署](../nano/README.md) | 服务、安装路径和运行产物 |
| [硬件与软件栈](../nano/docs/hardware.md) | 相机、IMU、串口及设备依赖 |
| [守护进程](../nano/docs/daemon.md) | 启动、内录、判活及重启条件 |
| [录制与地图](../nano/docs/recording-and-maps.md) | 连续状态、导航记录及按需图像内录 |
| [飞控接入](../nano/docs/flight-controller.md) | 当前配置入口、坐标与杆臂 |
| [五米任务流程](DECXIN_FIVE_METRE_FLIGHT.md) | 空中初始化、飞手拨档与 HOLD |
| [三档任务服务](../nano/docs/pilot-navigation.md) | 任务实现、状态和早期部署记录 |

## 标定与处理

| 文档 | 内容 |
|---|---|
| [处理分辨率与特征配置](RESOLUTION_PROFILES.md) | 960×720 常驻配置、全尺寸回退和性能对照 |
| [GPU 处理后端](GPU_PROCESSING.md) | GPU 校正、FAST、CPU 回退与验证 |
| [前端性能](FRONTEND_PERFORMANCE.md) | 特征提取与采集性能记录 |
| [DECXIN / PX4 接入记录](DECXIN_VIO_INTEGRATION_20261110.md) | 图像布局、时间与 IMU 接收实现 |
| [外参与时间标定](DECXIN_EXTRINSICS_20261110.md) | 旋转、尺量平移、时间偏移及独立验证 |

接入和标定记录保留各阶段的状态；当前服务入口与发布条件以 [五米任务流程](DECXIN_FIVE_METRE_FLIGHT.md) 为准。

## 开发与测试

| 文档 | 内容 |
|---|---|
| [构建说明](BUILDING.md) | 依赖、CMake 选项、Nano 与离线构建 |
| [Git 约定](GIT_CONVENTIONS.md) | 分支、提交、数据与行尾 |
| [测试命令汇总](../tests/TEST_COMMANDS.md) | 测试入口与输出位置，含早期 D435i 命令 |
| [导航坐标系](../tests/NAVIGATION_FRAME.md) | 位姿、速度、重力与参考变更 |
| [IMU 初始化参数](../tests/IMU_INITIALIZATION_CONFIG.md) | 运动门控累计阈值与测试 |
| [PX4 VIO 接入](../tests/PX4_VIO.md) | ROS 2 依赖、桥接和离线测试 |
| [PX4 验证](../tests/PX4_VALIDATION.md) | 桥接测试条件与结果 |
| [重复导航](REPEATED_NAVIGATION.md) | 地图复用及重复任务 |
| [电能记录](ENERGY_MONITOR.md) | 电池与电能监测 |

## 历史资料

以下文档保留原有设备、命令及实验条件，查阅时按记录日期和硬件区分。

| 文档或目录 | 内容 |
|---|---|
| [D435i 操作手册](../nano/docs/archive/d435i-manual.md) | 旧相机的编译、恢复测试与外参说明 |
| [D435i 飞控参数记录](../nano/docs/archive/d435i-flight-controller-20261105.md) | 2026-11-05 前后融合、时延与杆臂记录 |
| [Atlas 保存测试](../tests/README.md) | 上游恢复测试与 D435i 历史结果 |
| [录制回放](../tests/RECORD_REPLAY.md)、[终端与 SSH](../tests/REMOTE_AND_CONSOLE.md)、[回放性能](../tests/REPLAY_BENCHMARK.md) | 原恢复链路的操作与性能记录 |
| [2026-10-28 同步](AVI_SYNC_REPORT.md)、[10-30 同步](AVI_SYNC_2026-10-30.md)、[10-31 同步](AVI_SYNC_2026-10-31.md) | 功能来源及当时的验证结果 |
| [设备验证](../nano/verification/) | 按日期保存的部署、标定和测试证据 |
| [构建实施计划](plans/2026-11-08-build-profiles.md) | 构建调整的原始计划 |
| [上游 README](UPSTREAM_README.md)、[修改说明](LOCAL_CHANGES.md) | 上游使用方法与改动来源 |
