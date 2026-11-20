# Harrier 仿真配置与导航契约（2026-11-20）

本仓库维护 SLAM、导航快照和 PX4 发布契约；Gazebo/ROS 输入适配、飞手及行为树位于配套 RM27_drone_sim。当前同步增加 Harrier 仿真基线，核心实现沿用已有 master。不存在仅凭场景通过就放宽算法有效性或故障锁存的变更。

## 相机与 IMU 参数

`config/RM27_Harrier_IMU_Stereo_baseline.yaml` 对应当前 `x500_mono_cam` 中的 Harrier 安装几何：960 × 720 @ 30 Hz，HFOV 1.590 rad，`fx=fy=470.87`，双目基线 0.050 m。左目在 IMU/base_link FLU 中为 `(0.0485,0.025,-0.0057)` m，IMU 250 Hz。配置中的旋转把相机光学系转换到 IMU FLU，桥接再以 `R_BI` 转换为机体 FRD。

与通用 Gazebo 基线相比，Harrier 配置改变左目安装平移；相机内参、基线和惯性噪声仍来自仿真模型。初始化使用 VIBA1/2 累计阈值 3/10 s；`never_reset_map=0`、`retain_without_ba2=0`，另保留 `viba2_recovery_seconds=20` 的有界初始化地图策略。该策略不代表设备端配置一致，也不降低发布前 VIBA2 要求。本文件不是实机标定，不能替代 DECXIN 配置与实测外参。

## 跟踪、导航有效性与发布

`tracking_state==OK` 表示当前跟踪枚举；`GetNavigationState().valid` 还受导航参考与惯性初始化约束。接入端必须检查同一导航快照的有效性、速度有效性和参考身份，不能仅以跟踪枚举决定发布。原场地诊断观测到 3 帧 OK 但导航无效，这只能证明两种语义可能分叉，尚不足以唯一解释历史短时失锁。

`GetTrackedKeyPointsUn()` 复制的是当前帧去畸变 ORB 关键点容器，数量不是匹配内点数。输入节点 `pairs_hz/imu_hz` 为回调平均到达率，也不能代替逐帧时间同步和队列时龄检查。

首次建立有效导航参考与已经使用的参考发生切换分开处理。Gate 对时效、跟踪、速度及本地系复位继续执行原有检查；`px4_local_frame_reset` 在空中触发的 `requires_disarm` 锁存保留。地面重置、旧图加载与几何对齐流程不能据此解释为空中自动恢复。

## PX4 EV 时间链

PX4 v1.17.0 的 `EV_MAX_INTERVAL=200 ms`，超过两倍间隔会停用 EV 融合；再次启动水平位置融合时会复位参考。`no_aid_timeout_max=1 s` 是另一条 fusion-failing 判定，不存在可直接替代上述常量的 `EKF2_EV_TIMEOUT` 参数。

Gate 单次拒绝后的 1 s 稳定窗口可能跨过停融合边界。实际日志中的模式切换、fusion-failing 与重启融合有时间重叠，复位路径仍需逐事件区分。本次不改 `stable_seconds`、时龄、样本间隔、reset 锁存或安全包络，也不重发旧观测填补静默。

## 实验结果与边界

配套仿真正式组在 `rm27_midhangar_n_world`、出生位姿 `6.80,8.05,0.504900,0,0,1.5707963` 下执行 `bearing_deg=90` 的东向 5 m 任务。T1/T2 为 PASS，完成 BT HOLD、Position 接回与落地上锁；T3 未进入 NAVIGATE，交接前 `at_home` 未满足，60 s 超时退出，整组未通过。原始评价器 17 项中 15 项为必选；T1/T2 的非必选延迟证据仍为 UNVERIFIED。

六次运行的明细、候选源代码和构建入口见 [配套联调记录](https://github.com/MalzaharGeng/RM27_drone_sim/blob/master/docs/VIO_HARRIER_20261120.md)。本组不能外推原北向任务、长期可靠率或实飞可用性。后续需要解决交接前漂移、初始化裕度与低开销短时失锁捕获，再完成同一冻结候选三轮全通过。

本次同步检查：WSL 算法 CTest 20/20，核心源码与被测工作区去注释核对一致；Harrier 参数与冻结候选核对一致。配套仿真离线回归 117/117、EXCITE 生产回放 11/11、候选回归 24/24，三个 ROS 节点重新编译完成。本次未追加仿真飞行。
