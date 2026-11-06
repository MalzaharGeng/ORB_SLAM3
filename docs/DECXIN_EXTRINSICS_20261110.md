# DECXIN Camera / PX4 外参与时间偏移标定记录
采集平台：Jetson Orin Nano，Ubuntu 22.04 / aarch64 / ROS 2 Humble。

## 已完成的标定
- 相机为 DECXIN Camera，镜头向前、模组顶边向上。
- 尺量左目光心相对飞控 IMU：前 150 mm、右 20.5 mm、下 49.5 mm。
- 标定旋转将**校正后左目光学坐标**转到 PX4 body FRD，已包含 stereoRectify 的 R1。
- 平移采用尺量值，未用联合视觉惯性优化重新估计，不将其描述为自动标定结果。
- 时间偏移约 -44.534 ms：使用图像读出后主机时间戳减 44.534 ms，再与 IMU 对齐。
- 适用采集路径：MJPG 2720x960 @30 fps，单 V4L2 缓冲，读出后软件时间戳。
  改像素格式、缓冲、帧率后必须重做时间验证。当前程序拒绝已验证配置与 60 fps 混用。

## 数据与独立验证
1. motion1：105.35 秒，742 组图像，10469 条 IMU。排除了两段 IMU 缺口附近的区间。
2. motion2：51.62 秒后丢失飞控状态，356 组图像、4994 条 IMU。
   保留原始 incomplete 标记，仅诊断完整 IMU 覆盖的前缀，未作为最终确认数据。
   同期 CP2102 端点报 -32，随后 Nano 断连并重启；主机网口一度显示 Disconnected。
3. motion3：80.32 秒，557 组图像、8067 条 IMU。完整结束；最大 IMU 间隔 24.999 ms，无无效 IMU。

采用双目几何计算每对相邻帧的 PnP 相对旋转，再与 PX4 原始陀螺积分拟合，
联合估计旋转、软件时间偏移和短时陀螺偏置。没有用 PX4 融合姿态来反过来标定自己。
PnP 使用 C++ OpenCV 4.8.0 导出的运行时校正矩阵，避免 Python OpenCV 4.5.4 的 P1 差异。

motion1 与 motion3 的独立拟合：
- 时间偏移：-44.8096 / -44.5340 ms，相差 0.2755 ms。
- 旋转外参相差 0.3091 度。
- 第一轮参数预测第三轮：单个帧间旋转误差 p95 0.1528 度。
- 第三轮参数预测第一轮：单个帧间旋转误差 p95 0.0938 度。
这些是所列数据上的重复性/残差，不是所有场景的绝对精度保证。

## 生效配置
- Examples/Stereo-Inertial/DECXIN_Camera.yaml：同步实测 T_b_c1 与 100 Hz DDS IMU 频率。
- Examples/Stereo-Inertial/DECXIN_PX4_Bench.yaml：同一外参，整段/环形录制关闭用于台架。
- config/decxin_px4_imu.yaml：已验证旋转/时间 + 尺量平移，含采集路径匹配字段。
- 原始 JSON、每轮输入哈希、跨数据集验证结果：/home/nano/decxin_calibration_20261110/。
- 写入前备份：before_calibration_apply.tar.gz。
- 导航观测发布保持关闭；PX4 桥接仍为 receive-only，未发送解锁/模式/起飞命令。

## VIO 集成验证发现
惯性初始化期间曾多次因运动不足重置。静态启动复现 SIGSEGV，gdb 定位：
EdgePriorPoseImu -> PoseInertialOptimizationLastFrame -> Tracking::TrackLocalMap。
重定位后的视觉优化窗口不产生上一帧惯性先验，但窗口结束后代码直接选择依赖它的优化路径。
新增实际 TrackLocalMap 回归夹具构造缺失先验，修复前测试以 SEGFAULT 失败。
修复：当上一帧先验缺失时，先走现有 LastKeyFrame 惯性优化建立当前先验，再恢复 LastFrame 路径。
惯性初始化、VIBA2 和飞控发布门控均保持原来的质量要求。
修复后编译和缺先验回归测试通过；相关 5 项门控/缓冲/启动器检查通过。
第三轮数据 557 帧回放完成，无崩溃；实际命中并通过缺先验 bootstrap 分支。
该段以旋转为主的记录未完成 VIBA2，末帧 valid=1 / ba2=0，不能描述为完整 VIO 初始化通过。
修复后的实机持续平移验证完成 VIBA1 / VIBA2，随后持续静置保持 valid=1、imu=1、ba2=1，未换图、无跟踪丢失。
该次运行处理 1086 帧后因 waiting_imu 退出（exit 3），未崩溃。长时 IMU/DDS 连续性仍需完善。
因此：旋转/时间标定验证已完成，VIO 初始化已通过；整条导航链路的长时间稳定性尚未通过。
